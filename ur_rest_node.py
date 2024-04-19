"""REST-based client for the Liconic"""

import json
from argparse import ArgumentParser, Namespace
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import JSONResponse
import robot12idb as rb
from wei.core.data_classes import (
    ModuleAbout,
    ModuleAction,
    ModuleActionArg,
    ModuleStatus,
    StepResponse,
    StepStatus,
)
from wei.helpers import extract_version

global ur, state


def parse_args() -> Namespace:
    """Parses CLI args for the REST server

    Returns (ArgumentParser): Parsed arguments
    """
    parser = ArgumentParser()
    parser.add_argument("--name", type=str, default="UR arms", help="Module name")
    parser.add_argument(
        "--host", type=str, default="0.0.0.0", help="Host IP/Domain Name"
    )
    parser.add_argument("--port", type=str, default="3011", help="Port for REST API")
    parser.add_argument(
        "--ur_ip", type=str, default="UR5-12IDC.xray.aps.anl.gov", help="IP address of the UR robot"
    )
    return parser.parse_args()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initial run function for the app, initializes the state
    Parameters
    ----------
    app : FastApi
       The REST API app being initialized

    Returns
    -------
    None
    """

    global ur, state
    try:
        args = parse_args()
        # Do any instrument configuration here
        state = ModuleStatus.IDLE
        ur = ur.UR5(args.ur_ip)
    except Exception as err:
        print(err)
        state = ModuleStatus.ERROR

    # Yield control to the application
    yield

    # Do any cleanup here
    pass


app = FastAPI(
    lifespan=lifespan,
)


@app.get("/state")
def get_state():
    """Returns the current state of the module"""
    global ur, state

    if state not in [ModuleStatus.BUSY, ModuleStatus.ERROR]:
        rtn = ur.get_status()
        if "NORMAL" not in rtn:
            state = ModuleStatus.ERROR
        elif ur.get_movement_state() == "BUSY":
            state = ModuleStatus.BUSY
        else:
            state = ModuleStatus.IDLE

    return JSONResponse(content={"State": state})


@app.get("/about")
async def about():
    """Returns a description of the actions and resources the module supports"""
    global ur, state

    args = parse_args()
    # description = {
    #     "name": args.name,
    #     "type": "ur_arm",
    #     "actions": {
    #         "status": state,
    #         "pick_tool": "home, tool_loc, docking_axis, payload, tool_name",
    #         "place_tool": "home, tool_loc, docking_axis, payload, tool_name",
    #         "gripper_transfer": "home, source, target, source_approach_axis, target_approach_axis, source_approach_distance, target_approach_distance, gripper_open, gripper_close",
    #     },
    # }
    about = ModuleAbout(
        name=args.name,
        model="UR5,",
        description="UR robots are 6 degress of freedom manipulators. Different models of these robots allow to carry heavier payload and reach longer distances. This robot is mainly used in pick and place jobs",
        interface="wei_rest_node",
        version=extract_version(Path(__file__).parent.parent / "pyproject.toml"),
        actions=[
            ModuleAction(
                name="home",
                description="move and set the home position.",
                args=[
                    ModuleActionArg(
                        name="home",
                        description="home position.",
                        type="str",
                        required=True,
                    ),
                ],
            ),
            ModuleAction(
                name="loadsample",
                description="This action will load a sample from the storage to the ptychography setup.",
                args=[
                    ModuleActionArg(
                        name="pos_from",
                        description="Sample position at the storage.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="pos_to",
                        description="Target location to which the sample will go.",
                        type="str",
                        required=False,
                    ),
                ],
            ),
            ModuleAction(
                name="unloadsample",
                description="This action will unload a sample from the ptychography setup.",
                args=[
                    ModuleActionArg(
                        name="pos_to",
                        description="Target location to which the sample will go.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="pos_from",
                        description="Sample position of ptychography.",
                        type="str",
                        required=False,
                    ),
                ],
            ),
        ],
        resource_pools=[],
    )
    return JSONResponse(content=about.model_dump(mode="json"))


@app.get("/resources")
async def resources():
    """Returns the current resources available to the module"""
    global state
    return JSONResponse(content={"Resources": "TEST"})


@app.post("/action")
def do_action(
    action_handle: str,  # The action to be performed
    action_vars: str,  # Any arguments necessary to run that action
) -> StepResponse:
    """Runs the actions that are recieved
    Args
        action_handle (str): Action command
        action_vars (str): Action variable

    Returns (StepResponse): Response after action execution
    """
    global ur, state
    step_response = StepResponse(action_response=StepStatus.IDLE)

    if state == ModuleStatus.BUSY:
        step_response.action_response = StepStatus.FAILED
        step_response.action_log = "Module is busy"
    else:
        try:
            state = ModuleStatus.BUSY
            action_vars = json.loads(action_vars)

            if action_handle == "home":
                home = action_vars.get("home", None)

                if not home:
                    pass

                ur.home(
                    home=home,
                )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Robot sent to {home}",
                )


            elif action_handle == "loadsample":
                pos_from = action_vars.get("pos_from", None)
                pos_to = action_vars.get("pos_to", None)

                ur.loadsample(
                    pos_from=pos_from,
                    pos_to=pos_to,
                )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Load sample {pos_to} from {pos_from}",
                )

            elif action_handle == "unloadsample":
                pos_to = action_vars.get("pos_to", None)
                pos_from = action_vars.get("pos_from", None)

                ur.loadsample(
                    pos_to=pos_to,
                    pos_from=pos_from,
                )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Unload sample {pos_to} from {pos_from}",
                )


            elif action_handle == "run_urp_program":
                transfer_file_path = action_vars.get("transfer_file_path", None)
                program_name = action_vars.get("program_name", None)

                if not program_name:  # Return Fail
                    pass

                ur.run_urp_program(
                    transfer_file_path=transfer_file_path,
                    program_name=program_name,
                )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Run rup program {program_name}",
                )

            elif action_handle == "set_digital_io":
                channel = action_vars.get("channel", None)
                value = action_vars.get("value", None)

                if not channel:  # Return Fail
                    pass

                ur.set_digital_io(channel=channel, value=value)

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Digital IO, channel {channel} set for {value}",
                )
            else:
                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.FAILED,
                    action_msg="False",
                    action_log=f"Action {action_handle} not supported",
                )
        except Exception as e:
            print(str(e))
            state = ModuleStatus.ERROR
            step_response.action_response = StepStatus.FAILED
            step_response.action_log = str(e)
        else:
            state = ModuleStatus.IDLE
    return step_response


if __name__ == "__main__":
    """Tests"""
    import uvicorn

    args = parse_args()

    uvicorn.run(
        "ur_rest_node:app",
        host=args.host,
        port=int(args.port),
        reload=True,
    )