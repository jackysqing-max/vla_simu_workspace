"""Opt-in, round-fenced SAM evidence. Unconfigured legacy runs are unchanged."""
import json
import os
import time
from pathlib import Path
import numpy as np
from PIL import Image


def active_round():
    path = os.environ.get('BENCHMARK_ACTIVE_ROUND_FILE')
    if not path:
        return None
    try:
        value = json.loads(Path(path).read_text())
        return value if value.get('active') else False
    except (OSError, ValueError):
        return False


def same_round(before):
    after = active_round()
    if before is None:
        return after is None
    return bool(before and after and before['round_id'] == after['round_id'])


def save_inference(round_info, header, rgb, resized, masks, scores, indices,
                   union, prompt, start, end, node):
    if not round_info:
        return
    stamp = header.stamp.sec * 1000000000 + header.stamp.nanosec
    path = Path(round_info['directory']) / 'sam3_instances' / str(stamp)
    path.mkdir(parents=True, exist_ok=False)
    Image.fromarray(rgb).save(path / 'source_rgb.png')
    Image.fromarray(resized).save(path / 'model_input_rgb.png')
    Image.fromarray(union).save(path / 'published_union_mask.png')
    storage_masks, original_dtype = compact_masks(masks)
    np.savez_compressed(path / 'instance_masks_and_scores.npz',
                        masks=storage_masks, scores=scores, kept_indices=indices)
    (path / 'inference.json').write_text(json.dumps({
        'round_id': round_info['round_id'], 'stamp_ns': stamp,
        'frame_id': header.frame_id, 'prompt': prompt,
        'mask_original_dtype': original_dtype, 'mask_storage_dtype': str(storage_masks.dtype),
        'mask_storage_lossless': True, 'evidence_write_sec': time.time()-end,
        'model': 'facebook/sam3', 'start_wall_sec': start, 'end_wall_sec': end,
        'inference_and_postprocess_sec': end-start,
        'raw_instance_count': len(scores), 'kept_indices': indices.tolist(),
        'published_score': float(np.max(scores)) if len(scores) else 0.,
        'score_th': node.score_th, 'relative_score_th': node.relative_score_th,
        'mask_th': node.mask_th, 'max_instances': node.max_instances,
        'max_side': node.max_side, 'raw_masks_grid_hw': list(masks.shape[-2:]),
        'source_hw': list(rgb.shape[:2]),
        'score_policy': 'sidecar exact stamp; legacy Float32 topic remains unstamped'
    }, indent=2))


def mark_busy(info):
    control = os.environ.get('BENCHMARK_ACTIVE_ROUND_FILE')
    if not control:
        return
    path = Path(control).with_name('sam_inflight.json')
    if info:
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'round_id': info['round_id']}))
        tmp.replace(path)
    else:
        path.unlink(missing_ok=True)


def compact_masks(masks):
    """Losslessly pack binary postprocessed masks, retaining their source dtype.

    SAM3 returns 0/1 int64 masks here. Storing those as bool removes 8x I/O
    work without changing any mask value. Unexpected numeric masks stay intact.
    """
    source = np.asarray(masks)
    if source.dtype.kind in 'biu' and np.all((source == 0) | (source == 1)):
        return source.astype(np.bool_, copy=False), str(source.dtype)
    return source, str(source.dtype)
