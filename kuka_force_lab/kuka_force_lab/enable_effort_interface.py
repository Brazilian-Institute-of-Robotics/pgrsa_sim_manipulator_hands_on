import argparse
import os
import re
import shutil
import sys

TARGET_FILES = [
    "kr70_r2100_ros2_control.xacro",
    "kr70_r2100_2f85_ros2_control.xacro",
    "kr70_r2100_2f140_ros2_control.xacro",
]

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
JOINT5_VALUE = "0.3"
BACKUP_SUFFIX = ".pre_force_lab.bak"


def find_urdf_dir(explicit=None):

    if explicit:
        explicit = os.path.expanduser(explicit)
        if not os.path.isdir(explicit):
            return None
        if not any(os.path.exists(os.path.join(explicit, f)) for f in TARGET_FILES):
            return None
        return explicit
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("kuka_description")
        candidates.append(os.path.join(share, "urdf"))
        ws = os.path.abspath(os.path.join(share, "..", "..", "..", ".."))
        candidates.append(
            os.path.join(ws, "src", "KUKA-ROS2", "kuka_description", "urdf")
        )
    except Exception:  # noqa: BLE001
        pass
    home = os.path.expanduser("~")
    candidates.append(
        os.path.join(home, "kuka_ws", "src", "KUKA-ROS2", "kuka_description", "urdf")
    )

    for c in candidates:
        if os.path.isdir(c) and any(
            os.path.exists(os.path.join(c, f)) for f in TARGET_FILES
        ):
            # Prefere sempre um caminho dentro de src/.
            if os.sep + "src" + os.sep in c:
                return c
    for c in candidates:
        if os.path.isdir(c):
            return c
    return None


def joint_block_bounds(text, joint):
    """Início e fim do bloco <joint name="...joint_N"> ... </joint>."""
    m = re.search(r'<joint\s+name="[^"]*' + re.escape(joint) + r'"\s*>', text)
    if not m:
        return None
    end = text.find("</joint>", m.end())
    if end < 0:
        return None
    return m.start(), end


def has_effort(text, joint):
    b = joint_block_bounds(text, joint)
    if not b:
        return False
    block = text[b[0] : b[1]]
    return 'name="effort"' in block


def add_effort(text, joint):
    b = joint_block_bounds(text, joint)
    if not b or has_effort(text, joint):
        return text, False
    start, end = b
    block = text[start:end]
    m = re.search(r'([ \t]*)<state_interface\s+name="velocity"\s*/>', block)
    if m:
        indent = m.group(1)
        new_block = (
            block[: m.end()]
            + f'\n{indent}<state_interface name="effort"/>'
            + block[m.end() :]
        )
    else:
        new_block = (
            block.rstrip() + '\n        <state_interface name="effort"/>\n      '
        )
    return text[:start] + new_block + text[end:], True


def set_joint5_initial(text, value=JOINT5_VALUE):
    b = joint_block_bounds(text, "joint_5")
    if not b:
        return text, False
    start, end = b
    block = text[start:end]
    m = re.search(
        r'(<param\s+name="initial_value"\s*>)\s*([-\d.eE+]+)\s*(</param>)', block
    )
    if not m:
        return text, False
    if abs(float(m.group(2))) > 1e-6:
        return text, False  # já foi afastado de zero
    new_block = block[: m.start()] + m.group(1) + value + m.group(3) + block[m.end() :]
    return text[:start] + new_block + text[end:], True


def report(path, text):
    missing = [j for j in ARM_JOINTS if not has_effort(text, j)]
    b = joint_block_bounds(text, "joint_5")
    j5 = None
    if b:
        m = re.search(
            r'<param\s+name="initial_value"\s*>\s*([-\d.eE+]+)\s*</param>',
            text[b[0] : b[1]],
        )
        if m:
            j5 = float(m.group(1))
    return missing, j5


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Habilita a interface de effort e tira joint_5 da "
        "singularidade nos xacros de kuka_description."
    )
    ap.add_argument(
        "--urdf-dir",
        default=None,
        help="caminho de kuka_description/urdf (autodetectado)",
    )
    ap.add_argument(
        "--check", action="store_true", help="apenas verifica, sem alterar nada"
    )
    ap.add_argument(
        "--restore", action="store_true", help="restaura os arquivos a partir do backup"
    )
    ap.add_argument(
        "--skip-joint5", action="store_true", help="aplica só a interface de effort"
    )
    raw = list(argv if argv is not None else sys.argv[1:])
    if "--ros-args" in raw:
        raw = raw[: raw.index("--ros-args")]
    args, _ = ap.parse_known_args(raw)

    urdf = find_urdf_dir(args.urdf_dir)
    if not urdf:
        alvo = args.urdf_dir or "(autodetecção)"
        print("ERRO: não encontrei os xacros de ros2_control de " "kuka_description.")
        print(f"       Caminho tentado: {alvo}")
        print()
        print("       O diretório precisa existir E conter pelo menos um de:")
        for f in TARGET_FILES:
            print(f"         - {f}")
        print()
        print(
            "       Se o seu workspace não está em ~/kuka_ws, localize o "
            "caminho certo com:"
        )
        print(
            '         find ~ -name "kr70_r2100_2f85_ros2_control.xacro" ' "2>/dev/null"
        )
        print(
            "       e passe o diretório que aparecer (sem o nome do "
            "arquivo) em --urdf-dir."
        )
        return 1
    print(f"Diretório: {urdf}")
    if os.sep + "src" + os.sep not in urdf:
        print(
            "AVISO: este caminho parece ser a cópia INSTALADA (install/), "
            "não o source."
        )
        print(
            "       Editar aqui não sobrevive a um `colcon build`. "
            "Prefira src/KUKA-ROS2/kuka_description/urdf."
        )

    changed_any = False
    for fname in TARGET_FILES:
        path = os.path.join(urdf, fname)
        if not os.path.exists(path):
            print(f"  – {fname}: não existe neste workspace, ignorando.")
            continue

        if args.restore:
            bak = path + BACKUP_SUFFIX
            if os.path.exists(bak):
                shutil.copy2(bak, path)
                print(f"  ✓ {fname}: restaurado do backup.")
                changed_any = True
            else:
                print(f"  – {fname}: sem backup, nada a restaurar.")
            continue

        with open(path) as fh:
            text = fh.read()

        missing, j5 = report(path, text)
        if args.check:
            status = "OK" if not missing else f'FALTA effort em: {", ".join(missing)}'
            j5s = (
                "OK"
                if (j5 is not None and abs(j5) > 1e-6)
                else f"joint_5 initial_value = {j5} (SINGULARIDADE)"
            )
            print(f"  {fname}: {status} | {j5s}")
            continue

        new = text
        touched = []
        for j in ARM_JOINTS:
            new, did = add_effort(new, j)
            if did:
                touched.append(j)
        if not args.skip_joint5:
            new, did5 = set_joint5_initial(new)
            if did5:
                touched.append("joint_5.initial_value→" + JOINT5_VALUE)

        if new == text:
            print(f"  – {fname}: já estava correto, nada a fazer.")
            continue

        bak = path + BACKUP_SUFFIX
        if not os.path.exists(bak):
            shutil.copy2(path, bak)
        with open(path, "w") as fh:
            fh.write(new)
        changed_any = True
        print(f'  ✓ {fname}: {", ".join(touched)}')

    if args.check:
        return 0
    if changed_any:
        print()
        print(
            "Alterações aplicadas. Backups com sufixo "
            f'"{BACKUP_SUFFIX}" ficaram ao lado dos originais.'
        )
        print("AGORA É PRECISO recompilar e reiniciar o Gazebo:")
        print("    cd ~/kuka_ws && colcon build && source install/setup.bash")
        print("Depois confirme com:")
        print("    ros2 topic echo /joint_states --field effort --once")
        print("O array deve trazer números, não uma lista vazia nem NaN.")
    else:
        print("\nNada foi alterado — o workspace já estava correto.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
