"""URDF nominal-pose rigid-body inertia and sampled PD, no contact/plant claim."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RUN = ROOT / "simulation/test_results/20261002_classic_pd_first4_candidate"
URDF = RUN / "staging/go2_measured.urdf"
NAMES = [f"{leg}_{joint}_joint" for leg in ["lf", "rf", "lh", "rh"]
         for joint in ["hip", "upper_leg", "lower_leg"]]


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def vec(element, key="xyz", default="0 0 0"):
    return np.array([float(x) for x in (element.get(key, default) if element is not None else default).split()])


def origin(element):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", vec(element, "rpy")).as_matrix()
    t[:3, 3] = vec(element)
    return t


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class Model:
    def __init__(self, filename):
        tree = ET.parse(filename).getroot()
        self.links = {x.get("name"):x for x in tree.findall("link")}
        self.joints = {x.get("name"):x for x in tree.findall("joint")}
        self.children = {}
        for name, joint in self.joints.items():
            parent = joint.find("parent").get("link")
            self.children.setdefault(parent, []).append(name)
        root = set(self.links) - {x.find("child").get("link") for x in self.joints.values()}
        assert len(root) == 1
        self.root = root.pop()
        assert set(NAMES) == {name for name, x in self.joints.items() if x.get("type") != "fixed"}

    def fk(self, q, base=None):
        frames = {self.root:np.eye(4) if base is None else base}
        ancestors = {self.root:[]}
        axes = {}
        joint_origins = {}
        def visit(parent):
            for name in self.children.get(parent, []):
                joint = self.joints[name]
                child = joint.find("child").get("link")
                frame = frames[parent] @ origin(joint.find("origin"))
                chain = list(ancestors[parent])
                if joint.get("type") != "fixed":
                    axis = vec(joint.find("axis"), default="1 0 0")
                    axis /= np.linalg.norm(axis)
                    axes[name] = frame[:3, :3] @ axis
                    joint_origins[name] = frame[:3, 3].copy()
                    change = np.eye(4)
                    change[:3, :3] = Rotation.from_rotvec(axis * q[NAMES.index(name)]).as_matrix()
                    frame = frame @ change
                    chain.append(name)
                frames[child] = frame
                ancestors[child] = chain
                visit(child)
        visit(self.root)
        assert len(frames) == len(self.links)
        return frames, ancestors, axes, joint_origins

    def mass(self, q):
        frames, ancestors, axes, pivots = self.fk(q)
        mass = np.zeros((18, 18))
        components = []
        for name, link in self.links.items():
            inertial = link.find("inertial")
            if inertial is None:
                continue
            m = float(inertial.find("mass").get("value"))
            inertia = inertial.find("inertia")
            ix = {key:float(inertia.get(key)) for key in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]}
            local_I = np.array([[ix["ixx"], ix["ixy"], ix["ixz"]],
                                [ix["ixy"], ix["iyy"], ix["iyz"]],
                                [ix["ixz"], ix["iyz"], ix["izz"]]])
            frame = frames[name] @ origin(inertial.find("origin"))
            center, rotation = frame[:3, 3], frame[:3, :3]
            world_I = rotation @ local_I @ rotation.T
            jv, jw = np.zeros((3, 18)), np.zeros((3, 18))
            jv[:, :3] = np.eye(3)
            jv[:, 3:6] = -skew(center)
            jw[:, 3:6] = np.eye(3)
            for joint in ancestors[name]:
                index = 6 + NAMES.index(joint)
                jv[:, index] = np.cross(axes[joint], center - pivots[joint])
                jw[:, index] = axes[joint]
            mass += m * jv.T @ jv + jw.T @ world_I @ jw
            components.append(dict(link=name, mass=m, com=center, world_I=world_I, Jv=jv, Jw=jw))
        return mass, components


def finite_difference_mass(model, q, components, epsilon=1e-6):
    """Centered COM/rotation differences, separately from analytic Jacobian columns."""
    numeric = {x["link"]:dict(Jv=np.zeros((3,18)),Jw=np.zeros((3,18))) for x in components}
    for index in range(18):
        qs, bases = [], []
        for sign in [-1., 1.]:
            q1=q.copy(); base=np.eye(4)
            if index<3:base[index,3]=sign*epsilon
            elif index<6:
                axis=np.zeros(3);axis[index-3]=sign*epsilon
                base[:3,:3]=Rotation.from_rotvec(axis).as_matrix()
            else:q1[index-6]+=sign*epsilon
            qs.append(q1);bases.append(base)
        low=model.fk(qs[0],bases[0])[0];high=model.fk(qs[1],bases[1])[0]
        for component in components:
            name=component["link"];inertial=model.links[name].find("inertial")
            a=low[name]@origin(inertial.find("origin"));b=high[name]@origin(inertial.find("origin"))
            numeric[name]["Jv"][:,index]=(b[:3,3]-a[:3,3])/(2*epsilon)
            numeric[name]["Jw"][:,index]=Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).as_rotvec()/(2*epsilon)
    m=np.zeros((18,18));v_error=w_error=0.
    for c in components:
        jv=numeric[c["link"]]["Jv"];jw=numeric[c["link"]]["Jw"]
        m+=c["mass"]*jv.T@jv+jw.T@c["world_I"]@jw
        v_error=max(v_error,float(np.max(np.abs(jv-c["Jv"]))))
        w_error=max(w_error,float(np.max(np.abs(jw-c["Jw"]))))
    analytic=sum((c["mass"]*c["Jv"].T@c["Jv"]+c["Jw"].T@c["world_I"]@c["Jw"] for c in components),np.zeros((18,18)))
    return dict(epsilon=epsilon,max_COM_Jv_residual=v_error,max_world_rotation_Jw_residual=w_error,
        max_mass_matrix_residual=float(np.max(np.abs(m-analytic))),
        passed=v_error<2e-8 and w_error<2e-8 and float(np.max(np.abs(m-analytic)))<2e-8)


def held_pd_matrix(inertia, p, d, passive=.0):
    """4 one-ms semi-implicit steps, controller force frozen from sample start."""
    inv = np.linalg.inv(inertia)
    h = .001
    identity = np.eye(12)
    decay = identity - h * passive * inv
    a = np.block([[identity, h * decay], [np.zeros((12, 12)), decay]])
    b = np.vstack((h * h * inv, h * inv))
    held = np.zeros((24, 12))
    for n in range(4):
        held += np.linalg.matrix_power(a, n) @ b
    return np.linalg.matrix_power(a, 4) - held @ np.hstack((p * identity, d * identity))


def scalar_bounds(m, p, passive=.0):
    h = .001
    decay = 1. - h * passive / m
    a = np.array([[1., h * decay], [0., decay]])
    b = np.array([h * h / m, h / m])
    af = np.linalg.matrix_power(a, 4)
    g = sum((np.linalg.matrix_power(a, n) @ b for n in range(4)), np.zeros(2))
    adj = np.array([[af[1, 1], -af[0, 1]], [-af[1, 0], af[0, 0]]])
    ell = adj @ g
    lo = -(1. - np.linalg.det(af) + ell[0] * p) / g[1]
    hi = (1. + np.trace(af) + np.linalg.det(af) - (g[0] + ell[0]) * p) / (g[1] + ell[1])
    return float(lo), float(hi)


def evaluate(inertia):
    eigen = np.linalg.eigvalsh(inertia)
    report = dict(joint_eigen_inertias_kg_m2=eigen.tolist(), min_eigen_inertia=float(eigen[0]),
                  max_eigen_inertia=float(eigen[-1]), diagonal_inertias=np.diag(inertia).tolist())
    models = {}
    for passive in [0., .01]:
        gains = {}
        for p in [100., 220.982919]:
            bounds = [scalar_bounds(m, p, passive) for m in eigen]
            low, high = max(x[0] for x in bounds), min(x[1] for x in bounds)
            gains[str(p)] = dict(common_strict_D_interval=[low, high], interval_nonempty=low < high)
            d = 1. if p == 100. else 3.360637
            for label, value in [("original_or_profile_D", d), ("illustrative_mid_interval_D", (low + high) / 2)]:
                if label.startswith("illustrative") and low >= high:
                    continue
                f = held_pd_matrix(inertia, p, value, passive)
                poles = np.linalg.eigvals(f)
                gains[str(p)][label] = dict(D=value, spectral_radius=float(np.max(np.abs(poles))),
                    stable=bool(np.max(np.abs(poles)) < 1.), most_extreme_pole=[float(poles[np.argmax(np.abs(poles))].real),
                                                                             float(poles[np.argmax(np.abs(poles))].imag)])
        models[str(passive)] = gains
    report["semi_implicit_1ms_4step_hold"] = models
    report["exact_inertial_ZOH_4ms_no_passive"] = {}
    for p, d in [(100., 1.), (220.982919, 3.360637)]:
        t, inv = .004, np.linalg.inv(inertia)
        f = np.block([[np.eye(12) - .5 * t * t * p * inv, t * np.eye(12) - .5 * t * t * d * inv],
                      [-t * p * inv, np.eye(12) - t * d * inv]])
        report["exact_inertial_ZOH_4ms_no_passive"][str(p)] = dict(D=d,
            spectral_radius=float(np.max(np.abs(np.linalg.eigvals(f)))),
            common_strict_D_interval=[.5 * t * p, 2. * eigen[0] / t])
    report["explicit_comparison_profiles"]={}
    for label,p,d in [("original_100_D1",100.,1.),("failed_221_D3p36",220.982919,3.360637),
                      ("only_lower_D_221_D1",220.982919,1.),("unverified_comment_67p5_D0p3",67.5,.3)]:
        cases={}
        for passive in [0.,.01]:
            bounds=[scalar_bounds(m,p,passive) for m in eigen]
            lo=max(x[0] for x in bounds);hi=min(x[1] for x in bounds)
            poles=np.linalg.eigvals(held_pd_matrix(inertia,p,d,passive));rho=float(max(abs(poles)))
            cases["semi_implicit_passive_"+str(passive)]=dict(spectral_radius=rho,pole_margin=1-rho,
                stable=rho<1.,D_interval=[lo,hi],D_lower_margin=d-lo,D_upper_margin=hi-d)
        t=.004;inv=np.linalg.inv(inertia)
        f=np.block([[np.eye(12)-.5*t*t*p*inv,t*np.eye(12)-.5*t*t*d*inv],[-t*p*inv,np.eye(12)-t*d*inv]])
        rho=float(max(abs(np.linalg.eigvals(f))))
        lo=.5*t*p;hi=2*eigen[0]/t
        cases["exact_inertial_ZOH_no_passive"]=dict(spectral_radius=rho,pole_margin=1-rho,stable=rho<1.,
            D_interval=[lo,hi],D_lower_margin=d-lo,D_upper_margin=hi-d)
        report["explicit_comparison_profiles"][label]=dict(P=p,D=d,models=cases)
    return report


def main():
    assert sha(URDF) == "c1d454f479246a095e9f90597213674c4abc28a7661c0573dca78e43bd0e7c7f"
    adapter = RUN / "joint_stop_adapter.jsonl"
    nominal = None
    with adapter.open() as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            if row["kind"] == "nominal_frozen":
                nominal = row
                break
    assert nominal is not None and line_number == 3649
    q = np.array(nominal["positions"])
    model = Model(URDF)
    mass, components = model.mass(q)
    fixed = mass[6:, 6:]
    schur = fixed - mass[6:, :6] @ np.linalg.solve(mass[:6, :6], mass[:6, 6:])
    checks = dict(full18_matrix_symmetric=float(np.max(np.abs(mass - mass.T))) < 1e-12,
        full18_matrix_positive=float(np.linalg.eigvalsh(mass)[0]) > 0,
        floating_Schur_positive=float(np.linalg.eigvalsh(schur)[0]) > 0,
        total_mass_translation_block=np.allclose(mass[:3, :3], sum(x["mass"] for x in components) * np.eye(3), atol=1e-12),
        fixed_joint_cross_leg_blocks_zero=all(np.max(np.abs(fixed[a:a + 3, b:b + 3])) < 1e-12
            for a in range(0, 12, 3) for b in range(0, 12, 3) if a != b))
    rng = np.random.default_rng(20261002)
    residuals = []
    for _ in range(8):
        speed = rng.normal(size=18)
        direct = sum(x["mass"] * np.linalg.norm(x["Jv"] @ speed) ** 2
            + (x["Jw"] @ speed) @ x["world_I"] @ (x["Jw"] @ speed) for x in components)
        residuals.append(abs(direct - speed @ mass @ speed))
    checks["kinetic_energy_sum"] = max(residuals) < 1e-10
    fd=finite_difference_mass(model,q,components)
    checks["finite_difference_18_columns"]=fd["passed"]
    frames_path=ROOT/"simulation/test_results/classic_pd_startup_audit/native_100s_short_frames.json"
    frames=json.loads(frames_path.read_text())
    actual=next(x for x in frames if x["kind"]=="jtc" and x["header"]["header_stamp_ns"]==100000000000)
    measured=dict(zip(actual["joint_names"],actual["values"]["feedback.positions"]))
    assert set(measured)==set(NAMES)
    actual_q=np.array([measured[name] for name in NAMES]);actual_mass,actual_components=model.mass(actual_q)
    actual_fixed=actual_mass[6:,6:]
    actual_schur=actual_fixed-actual_mass[6:,:6]@np.linalg.solve(actual_mass[:6,:6],actual_mass[:6,6:])
    actual_fd=finite_difference_mass(model,actual_q,actual_components)
    checks["actual_pose_finite_difference_18_columns"]=actual_fd["passed"]
    checks["actual_pose_joint_matrices_positive"]=float(np.linalg.eigvalsh(actual_schur)[0])>0
    np.savez(HERE/"actual_100s_mass_matrices.npz",q=actual_q,full18=actual_mass,
        base_fixed12=actual_fixed,floating_Schur12=actual_schur)
    result = dict(scope=__doc__, source_URDF=str(URDF), source_URDF_sha256=sha(URDF),
        nominal_source=dict(path=str(adapter), line=line_number, record=nominal, joint_names=NAMES,
            mapping_source="simulation/joint_stop_core.py JointGeometry.names; not sensor message order"),
        generalized_velocity_order=["base_linear_x", "base_linear_y", "base_linear_z", "base_angular_x",
            "base_angular_y", "base_angular_z", *NAMES], root_link=model.root,
        links=len(model.links), massive_links=len(components), total_mass_kg=sum(x["mass"] for x in components),
        checks={key:bool(value) for key, value in checks.items()}, kinetic_energy_max_residual=max(residuals),
        fixed_base=evaluate(fixed), floating_base_Schur=evaluate(schur), finite_difference=fd,
        actual_100s_pose=dict(source=str(frames_path),sha256=sha(frames_path),header=actual["header"],
            source_names=actual["joint_names"],mapped_names=NAMES,mapped_q=actual_q.tolist(),
            max_difference_from_nominal=float(max(abs(actual_q-q))),finite_difference=actual_fd,
            fixed_base=evaluate(actual_fixed),floating_base_Schur=evaluate(actual_schur)),
        single_fixed_leg_eigen_inertias=np.linalg.eigvalsh(fixed[:3, :3]).tolist(),
        sampled_model=dict(controller_period_s=.004, physics_step_s=.001, frozen_effort_substeps=4,
            no_passive_common_D_bounds="1.5e-3*P < D < 500*m_min - 5e-4*P",
            exact_ZOH_common_D_bounds="2e-3*P < D < 500*m_min",
            friction_not_linearized=.2, declared_URDF_joint_damping=.01),
        limitations=["No foot contacts, constraint solver, impacts, Coulomb friction, gravity-linearization stiffness, Coriolis, actuator velocity/effort saturation or target interpolation is included.",
            "The 1ms semi-implicit integrator is an explicit model assumption, not a verification of Gazebo/DART internal integration or actual closed-loop dynamics.",
            "Floating Schur eliminates force-free base acceleration. Fixed base holds the entire body. Neither is the actual four-foot contact plant.",
            "Illustrative interior D values are spectral checks of this frozen rigid-body model only, not proposed or adopted control gains.",
            "Native 200/250Hz prescribed e/v response fitting is not a Go2 inertia or physical-plant fit.",
            "Original JTC=false includes stateful interpolation/reference effort carry; direct-PD 100/1 spectra do not model that actual controller.",
            "P67.5/D0.3 is an unverified old configuration comment, included only as a numerical contrast; it is not recommended or adopted.",
            "Nominal and one actual 100s pose are two frozen snapshots, not a certified bound over the full motion/contact workspace.",
            "I=.2 is not in this PD-only spectral calculation; no integral, saturation, measurement delay, target scheduling or stability guarantee is claimed."])
    np.savez(HERE / "nominal_mass_matrices.npz", q=q, full18=mass, base_fixed12=fixed, floating_Schur12=schur)
    (HERE / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(dict(checks=result["checks"], total_mass_kg=result["total_mass_kg"],
        single_leg=result["single_fixed_leg_eigen_inertias"],
        nominal={key:result[key]["explicit_comparison_profiles"] for key in ["fixed_base","floating_base_Schur"]},
        actual={key:result["actual_100s_pose"][key]["explicit_comparison_profiles"] for key in ["fixed_base","floating_base_Schur"]}), indent=2))


if __name__ == "__main__":
    main()
