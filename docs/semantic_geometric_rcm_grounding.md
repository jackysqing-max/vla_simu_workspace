# Semantic-Geometric RCM Grounding

This version removes language interpretation from the RGB-D port pose node. The old
path used keyword parsing such as left/right/up/down and fell back to the largest
hole when language was unresolved. That was useful for a fixed nine-hole demo, but
it is not a zero-shot grounding interface: new spatial descriptions, exclusions,
scene objects, and feasibility preferences cannot be represented by a closed
row/column table.

The new runtime separates responsibilities:

1. `vlm_port_pose_node` receives the SAM3 mask, extracts every visible recessed
   aperture candidate, estimates center/axis/quality fields, and publishes the
   dynamic set on `/vlm_rcm/hole_candidates` and `/vlm_rcm/candidates`.
2. `semantic_port_grounder_node` receives the original instruction, the marked
   candidate overlay image, and candidate features. It calls a Qwen-VL-compatible
   chat-completions endpoint and asks for an objective, candidate ranking, and a
   single restricted scoring expression.
3. `semantic_geometric_verifier.py` checks the model output against the current
   candidate set. It returns exactly `ACCEPT`, `REQUERY`, or `REJECT`.
4. Only an `ACCEPT` message on `/vlm_rcm/verified_selected_port` allows
   `vlm_port_pose_node` to lock and publish `/vlm_rcm/locked_port_point`,
   `/vlm_rcm/locked_port_axis`, and `/vlm_rcm/port_ready`.
5. `surgical_rcm_task_executor_node` consumes a high-level RCM task request. It
   still uses fixed internal safety stages, but those stages are no longer
   serialized as if the LLM had planned them.
6. After the port locks, `rcm_virtual_fixture_node` treats its inward axis as the
   center of an admissible approach cone. It samples continuous-axis candidates,
   checks the pre-insertion, port-standoff, and inserted poses with IK plus FK
   error gates, and freezes one axis before motion starts.

The candidate IDs are observation-dependent labels such as `H1`, `H2`, `H10`.
They are not semantic slots and the pipeline does not assume exactly nine holes.
Each candidate carries pixel center, world center, inward axis, surface normal,
aperture area, recessed-depth checks, circularity, plane RMS, perception quality,
temporal stability, and deterministic feasibility fields. IK/collision/risk are
currently explicit deterministic fields with `not_evaluated` sources, so later
robot-model checks can reject candidates without changing the VLM interface.

The scoring program is intentionally restricted. The model may produce an
expression such as:

```text
return -distance_to(target) + 2.0 * clearance_to(vessel) - semantic_risk()
```

The system never runs model text with `eval()` or `exec()`. The compiler in
`port_grounding_expression.py` interprets a small AST subset: numeric constants,
arithmetic, comparisons, boolean combinations, approved candidate features, and a
registered geometric function library. It rejects imports, arbitrary attributes,
file/network/subprocess access, loops, comprehensions, mutation, and Python
built-ins.

Qwen-VL decides the open semantic preference: which candidate IDs match the
instruction, what evidence supports the ranking, whether the request is
ambiguous, whether alternatives are allowed, and what expression represents the
objective. Deterministic code decides whether the candidate exists, whether the
expression can be independently evaluated, whether ranking and expression agree,
whether the selected candidate is stable, and whether geometry/RCM/IK/collision
fields permit execution.

Important topics:

```text
/vlm_rcm/hole_candidates
/vlm_rcm/candidates
/vlm_rcm/candidate_overlay
/vlm_rcm/language_command
/vlm_rcm/semantic_grounding_request
/vlm_rcm/semantic_hypothesis
/vlm_rcm/verification_result
/vlm_rcm/verified_selected_port
/vlm_rcm/locked_port_point
/vlm_rcm/port_ready
/rcm_virtual_fixtures/approach_constraint_json
/rcm_virtual_fixtures/approach_selection_json
```

Launch perception-only grounding:

```bash
QWEN_VL_MODEL=Qwen/Qwen3-VL-4B-Instruct \
QWEN_VL_API_BASE_URL=http://127.0.0.1:8000/v1/chat/completions \
./start_vlm_rcm_port_perception_demo.sh start

./start_vlm_rcm_port_perception_demo.sh locate "Use the upper-left visible port."
```

If Qwen-VL is not reachable, the verifier publishes `REQUERY` and the pose node
will not lock a fallback port. That is deliberate: unresolved semantics should be
visible during debugging, not silently converted into a largest-hole selection.

The high-level LLM task request for the full demo now looks like:

```json
{
  "instruction": "Use the safest feasible port near the lesion.",
  "objective_text": "Prefer clearance while keeping the target reachable.",
  "selected_candidate_id": null,
  "exclusions": [],
  "allow_alternative": false,
  "terminal_operation": "execute_rcm_circle",
  "requested_insertion_depth_m": 0.0,
  "approach_preference": "",
  "approach_constraint": {
    "mode": "auto_closest_reachable",
    "cone_half_angle_deg": 20.0,
    "preferred_tilt_deg": null,
    "preferred_azimuth_deg": null
  },
  "verification": {
    "decision": "PENDING"
  }
}
```

The finite controller stages remain inside the executor because they are a safety
boundary. They do not reduce the open semantic grounding problem to a fixed
nine-cell ontology.

For `auto_closest_reachable`, the deterministic controller chooses the reachable
cone axis whose pre-insertion IK solution has the smallest RMS joint displacement
from the current robot state. If the instruction explicitly contains numerical
tilt and azimuth, Qwen emits `preferred_cone_angle`; the controller searches for
the closest reachable direction to that preference while keeping it inside the
cone. Directional words are not converted by local keyword tables. The PyBullet
GUI draws the outward-opening admissible cone in orange and the chosen approach
axis in blue. `NO_REACHABLE_AXIS` blocks motion. Current feasibility covers joint
limits and kinematic tracking errors; environment collision checking is still a
separate required gate before transferring this demo to physical hardware.
