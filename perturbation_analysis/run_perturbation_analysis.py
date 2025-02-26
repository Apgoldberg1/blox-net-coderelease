import pybullet as p
import pybullet_data
from bloxnet.structure import Assembly
from perturbation_analysis.monte_carlo import run_monte_carlo
from perturbation_analysis.perturbation_utils import needs_perturbation
from PIL import Image
from bloxnet.pybullet.pybullet_images import get_imgs
from bloxnet.utils.utils import slugify


def _setup_scene():
    if not p.isConnected():
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    plane = p.loadURDF("plane.urdf")  # Load plane (ground)
    p.setGravity(0, 0, -9.81)


def run_perturbation_analysis(assembly):
    """
    Steps to follow:
    1. Figure out which blocks are aligned --> circular and rectangular edges/faces
    2. Jiggle ONLY UNSTABLE blocks with monte carlo, but blocks that are aligned should be jiggled together
        Unstable construction gets a 0 (collision or falling over), stable gets a 1
        If jiggling doesn't lead to convergence, try a larger range for monte carlo and go again
    3. Find the average of all the stable points
        Look for a point that has a stable radius of 3mm --> no point in a 3mm circle around it is unstable (ideally)
    """
    _setup_scene()

    for _ in range(3):
        for block in assembly.structure.structure:
            if needs_perturbation(block, assembly):
                print(block)
                assembly = run_monte_carlo(
                    block, assembly, num_circles=4, num_points_per_circle=8
                )
            else:
                pass

    assembly.structure.place_blocks(drop=False)

    final_img = Image.fromarray(get_imgs(keys=["isometric"], axes=True, labels=False))
    assembly.pre_perturbation_image = assembly.isometric_image
    assembly.isometric_image = final_img
    return assembly

def perturb_object(object):
    print (f"Running perturbation on {object}")
    try:
        assembly = Assembly.load(f"gpt_caching/{slugify(object)}/best_assembly/assembly.pkl")
        assembly = run_perturbation_analysis(assembly)
        assembly.human_friendly_save(f"perturbation_analysis/perturbation_analysis/{slugify(object)}")
    except Exception as e:
        print("-----------------")
        print(f"{object} failed, {e}")
        print("-----------------")

if __name__ == "__main__":
    assembly = Assembly.load("gpt_caching/rook-chess-piece/best_assembly/assembly.pkl")
    assembly = run_perturbation_analysis(assembly)
    assembly.human_friendly_save("perturbation_analysis/test")
