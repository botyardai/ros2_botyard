"""Open this environment's scene inside the running Isaac Sim application."""

import asyncio
from pathlib import Path

import carb
import omni.kit.app
import omni.usd

SCENE = Path(__file__).resolve().with_name("right_hand.usd")


async def open_scene():
    await omni.kit.app.get_app().next_update_async()
    success, error = await omni.usd.get_context().open_stage_async(str(SCENE))
    if not success:
        carb.log_error(f"Failed to open {SCENE}: {error}")
        return
    print(f"Scene loaded: {SCENE}. Click Play to start simulation.", flush=True)


_scene_load_task = asyncio.ensure_future(open_scene())
