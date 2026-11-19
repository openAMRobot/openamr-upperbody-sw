# SPDX-License-Identifier: MIT
"""Read simulation/parameter_register.yaml and evaluate its checks against the Gazebo, MuJoCo and Isaac representations.

Each check names a simulator and a reader. A reader extracts one value from one representation:
  gazebo      the expansion text (the URDF Gazebo gets) through urdf_model.URDF, plus its <gazebo> plugin and sensor tags
  isaac_urdf  the importer URDF of the Isaac job (the expansion with absolute mesh paths); isaac reads isaac_job.json
  mujoco      the compiled MjModel of the generated MJCF, and the generator report
The expected value is the row's value, or the check's `expect`, or, for a declared deviation, the deviation's value.
Nothing here writes a model or changes a value.
"""
import math
import pathlib

import numpy as np
import yaml

from .urdf_model import URDF, rpy_matrix

REGISTER = pathlib.Path(__file__).resolve().parents[1] / "parameter_register.yaml"
REQUIRED = ("quantity", "unit", "value", "verification", "source", "authority", "checks", "validation", "owner", "calibration")


def load(path=REGISTER):
    return yaml.safe_load(pathlib.Path(path).read_text())


def lookup(data, keys):
    for k in keys:
        data = data[k]
    return data


def pose_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(pose["rpy"])
    T[:3, 3] = pose["xyz"]
    return T


class Representations:
    """The three simulator representations of one configuration."""

    def __init__(self, expansion_text, mj_model, report, isaac_job, isaac_urdf_text):
        import mujoco
        self.mj = mujoco
        self.urdf = URDF(expansion_text)
        self.isaac_urdf = URDF(isaac_urdf_text)
        self.m = mj_model
        self.d = mujoco.MjData(mj_model)
        mujoco.mj_forward(self.m, self.d)
        self.report = report
        self.job = isaac_job

    # ---------------------------------------------------------------- URDF readers (gazebo and isaac_urdf)
    @staticmethod
    def _urdf(u, c):
        read = c["read"].split(".", 1)[1]
        if read == "joint":
            j = u.joints[c["joint"]]
            f = c["field"]
            if f == "z":
                return float(j.origin[2, 3])
            if f == "axis":
                return [float(v) for v in j.axis]
            if f == "type":
                return j.type
            if f == "limits":
                return [j.lower, j.upper]
            return getattr(j, f)
        if read == "collision":
            g = u.links[c["link"]].collisions[c.get("index", 0)]
            return g.params[c["field"]]
        if read == "mass":
            return u.links[c["link"]].inertial.mass
        if read == "total_mass":
            return u.total_mass()
        if read == "inertia_diagonal":
            inertia = u.links[c["link"]].inertial.inertia
            return [float(inertia[0, 0]), float(inertia[1, 1]), float(inertia[2, 2])]
        if read == "distance":
            fk = u.fk()
            return float(np.linalg.norm(fk[c["a"]][:3, 3] - fk[c["b"]][:3, 3]))
        if read == "pose":
            fk = u.fk()
            return np.linalg.inv(fk[c["relative_to"]]) @ fk[c["frame"]]
        if read == "lowest_point":
            # Lowest point of the link's first collision shape in the root frame at the zero pose (cylinder axis horizontal).
            fk = u.fk()
            g = u.links[c["link"]].collisions[0]
            T = fk[c["link"]] @ g.origin
            if g.kind == "sphere":
                return float(T[2, 3] - g.params["radius"])
            if g.kind == "cylinder":
                axis = T[:3, 2]
                if abs(axis[2]) > 1e-6:
                    raise ValueError(f"{c['link']}: cylinder axis is not horizontal")
                return float(T[2, 3] - g.params["radius"])
            raise ValueError(f"{c['link']}: lowest point of a {g.kind} is not supported")
        raise KeyError(c["read"])

    def _gazebo_tags(self, c):
        read = c["read"].split(".", 1)[1]
        if read == "plugin":
            found = [p for g in self.urdf.gazebo for p in g.iter("plugin") if p.get("filename") == c["plugin"]]
            if len(found) != 1:
                raise ValueError(f"{len(found)} plugins {c['plugin']} in the expansion")
            return float(found[0].find(c["tag"]).text)
        if read == "sensor_on":
            return sorted(s.get("name") for g in self.urdf.gazebo if g.get("reference") == c["frame"] for s in g.iter("sensor"))
        raise KeyError(c["read"])

    # ---------------------------------------------------------------- MuJoCo readers
    def _id(self, kind, name):
        i = self.mj.mj_name2id(self.m, kind, name)
        if i < 0:
            raise KeyError(f"{name} is not in the MuJoCo model")
        return i

    def _body_T(self, name):
        b = self._id(self.mj.mjtObj.mjOBJ_BODY, name)
        T = np.eye(4)
        T[:3, :3] = self.d.xmat[b].reshape(3, 3)
        T[:3, 3] = self.d.xpos[b]
        return T

    def _mujoco(self, c):
        obj = self.mj.mjtObj
        read = c["read"].split(".", 1)[1]
        if read == "geom_size":
            return float(self.m.geom_size[self._id(obj.mjOBJ_GEOM, c["geom"])][c["index"]])
        if read == "body_distance":
            return float(np.linalg.norm(self._body_T(c["a"])[:3, 3] - self._body_T(c["b"])[:3, 3]))
        if read == "body_offset":
            T = np.linalg.inv(self._body_T(c["relative_to"])) @ self._body_T(c["body"])
            return float(T["xyz".index(c["field"]), 3])
        if read == "pose":
            return np.linalg.inv(self._body_T(c["relative_to"])) @ self._body_T(c["body"])
        if read == "joint":
            j = self._id(obj.mjOBJ_JOINT, c["joint"])
            f = c["field"]
            if f == "axis":
                return [float(v) for v in self.m.jnt_axis[j]]
            if f == "range":
                return [float(v) for v in self.m.jnt_range[j]]
            if f == "type":
                return {int(self.mj.mjtJoint.mjJNT_HINGE): "hinge", int(self.mj.mjtJoint.mjJNT_SLIDE): "slide",
                        int(self.mj.mjtJoint.mjJNT_FREE): "free", int(self.mj.mjtJoint.mjJNT_BALL): "ball"}[int(self.m.jnt_type[j])]
        if read == "actuator_forcerange":
            a = self._id(obj.mjOBJ_ACTUATOR, c["actuator"])
            lo, hi = (float(v) for v in self.m.actuator_forcerange[a])
            return {"upper": hi, "lower": lo}[c["field"]]
        if read == "mass":
            return float(self.m.body_mass[self._id(obj.mjOBJ_BODY, c["body"])])
        if read == "total_mass":
            return float(sum(self.m.body_mass))
        if read == "inertia_principal":
            return sorted(float(v) for v in self.m.body_inertia[self._id(obj.mjOBJ_BODY, c["body"])])
        if read == "camera":
            k = self._id(obj.mjOBJ_CAMERA, c["camera"])
            if c["field"] == "fovy":
                return float(self.m.cam_fovy[k])
            if c["field"] == "body":
                return self.mj.mj_id2name(self.m, obj.mjOBJ_BODY, int(self.m.cam_bodyid[k]))
        if read == "report":
            return lookup(self.report, c["path"])
        if read == "unsupported":
            hits = [u for u in self.report["unsupported"] if u["item"] == c["item"] and u["where"] == c["where"]]
            if len(hits) != 1:
                raise ValueError(f"{len(hits)} unsupported entries for {c['item']} at {c['where']}")
            return hits[0]["value"]
        raise KeyError(c["read"])

    def _isaac(self, c):
        read = c["read"].split(".", 1)[1]
        if read == "drive":
            return self.job["drives"][c["joint"]][c["field"]]
        raise KeyError(c["read"])

    def read(self, c):
        family = c["read"].split(".", 1)[0]
        if family == "urdf":
            value = self._urdf(self.urdf, c)
        elif family == "isaac_urdf":
            value = self._urdf(self.isaac_urdf, c)
        elif family == "gazebo":
            value = self._gazebo_tags(c)
        elif family == "mujoco":
            value = self._mujoco(c)
        elif family == "isaac":
            value = self._isaac(c)
        else:
            raise KeyError(c["read"])
        if "scale" in c and value is not None:
            value = value * c["scale"]
        return value


def expected(row, check):
    if "deviation" in check:
        return check["deviation"]["value"]
    if "expect" in check:
        return check["expect"]
    return row["value"]


def nominal(row, check):
    """The value the check would have without a declared deviation."""
    return check["expect"] if "expect" in check else row["value"]


def same(actual, want, tol):
    """Equality of numbers, lists, strings, None and poses (a 4x4 actual against an xyz/rpy want)."""
    if isinstance(actual, np.ndarray) and actual.shape == (4, 4):
        W = pose_matrix(want)
        return bool(np.allclose(actual[:3, 3], W[:3, 3], atol=tol, rtol=0) and np.allclose(actual[:3, :3], W[:3, :3], atol=tol, rtol=0))
    if want is None or actual is None:
        return actual is None and want is None
    if isinstance(want, dict) or isinstance(actual, dict):
        return False
    if isinstance(want, str) or isinstance(actual, str):
        return actual == want
    if isinstance(want, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(want):
            return False
        return all(same(a, w, tol) for a, w in zip(actual, want))
    return math.isclose(float(actual), float(want), rel_tol=0.0, abs_tol=tol)


def describe(actual):
    if isinstance(actual, np.ndarray) and actual.shape == (4, 4):
        R = actual[:3, :3]
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
        return {"xyz": [round(float(v), 9) for v in actual[:3, 3]], "rpy": [round(v, 9) for v in (roll, pitch, yaw)]}
    return actual
