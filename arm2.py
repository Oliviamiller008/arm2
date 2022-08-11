import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions, dial_direct
from viam.components.arm import Arm, WorldState, Pose, JointPositions
from viam.components.gripper import Gripper
from viam.services.motion import MotionClient
from viam.services.types import ServiceType
from viam.proto.api.common import Pose, PoseInFrame, WorldState, GeometriesInFrame, Geometry, RectangularPrism

async def client():
    creds = Credentials(
        type='robot-location-secret',
        payload='lauqrt4op4x6ubhji867wue0qsnqdq76x00ljfv7vcoyq7mi')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    async with await RobotClient.at_address(
    'FriendArm2-main.rdt5n4brox.local.viam.cloud:8080',
    opts) as robot:
        print('Resources:')
        print(robot.resource_names)

    arm = Arm.from_robot(robot, 'arm')
    gripper = Gripper.from_robot(robot, 'vg1')
    vision = robot.get_service(ServiceType.VISION)
    motion = robot.get_service(ServiceType.MOTION)

    # initalize motion planning info
    geom = Geometry(center=Pose(x=-500, y=0, z=-190), box=RectangularPrism(width_mm =2000, length_mm =2000, depth_mm =10))
    geomFrame = GeometriesInFrame(reference_frame="arm_offset", geometries=[geom])
    worldstate = WorldState(obstacles=[geomFrame])

    for resname in robot.resource_names:
        if resname.name == "arm":
            armRes = resname

    # position = await arm.get_end_position()
    # arm_position = PoseInFrame(reference_frame = "arm_offset", pose=position)
    # transform = await robot.transform_pose(query=arm_position, destination="world")
    # print(transform)

    # return

    # pose = Pose(x=300, y=-500, z=200, o_x=0, o_y=0, o_z=-1)
    # world_pose = PoseInFrame(reference_frame="world", pose=pose)
    # await motion.move(component_name=armRes, destination=world_pose, world_state=worldstate)

    # return

    names = await vision.get_detector_names()
    print(names)

    detect_strawberry = await vision.get_detections_from_camera('cam', 'detector_color')
    print(detect_strawberry)

    count=0
    
    #pixel to mm conversions
    YCONV = 1.1
    XCONV = 1.45

    # probably some type of loop starting here
    while(len(detect_strawberry)!=0):

        y = (detect_strawberry[0].x_max + detect_strawberry[0].x_min)/2
        x = (detect_strawberry[0].y_max + detect_strawberry[0].y_min)/2

        # move to above the strawberry
        cam_pose = Pose(x=-x*XCONV, y=-y*YCONV, z=200, o_x=0, o_y=0, o_z=-1)
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)


        # move down, grab, move back up
        await gripper.open()
        cam_pose.z -= 188
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)
        await gripper.grab()
        await asyncio.sleep(1)
        await gripper.stop()
        cam_pose.z += 300
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)


        # look for chocolate detections
        #detect_chocolate = await vision.get_detections_from_camera("cam", "detector_chocolate")
        # print(detect_chocolate)


        #chocolate pot location
        x_choc = 250 #(detect_chocolate[0].x_max - detect_chocolate[0].x_min) + detect_chocolate[0].x_min
        y_choc = 500 #(detect_chocolate[0].y_max - detect_chocolate[0].y_min) + detect_chocolate[0].y_min

        # move to above the chocolate
        choc_pose = Pose(x=x_choc*XCONV, y=-y_choc*YCONV, z=300, o_x=0, o_y=0, o_z=-1) # change z based on chocolate warmer height
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)


        # dip
        choc_pose.z -= 200
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)
        await asyncio.sleep(1)
        choc_pose.z += 200
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)


        # move to plate pose
        plate_pose = Pose(x=362, y=-1000, z=200, o_x=0, o_y=0, o_z=-1) # move down with z
        plate_pose.x += (10*count)
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)

        #move down to drop on the plate
        plate_pose.z -= 50
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)

        # open gripper
        await gripper.open()
        await asyncio.sleep(1)
        await gripper.stop()

        #move gripper up and over
        plate_pose.z += 200
        plate_pose.y+=200
        await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)

        detect_strawberry = await vision.get_detections_from_camera("cam", "detector_color")
        print(detect_strawberry)
        count+=1

    await robot.close()


if __name__ == '__main__':
  asyncio.run(client())
