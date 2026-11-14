"""Single-request, state-checked controller switch for the isolated simulator.

Only the acceptance runner invokes this helper, inside its owned ROS domain and
Gazebo partition. Never retry a mutating service call after a lost response.
"""


def transition(query, request, target):
    if target not in ('active', 'inactive'):
        raise ValueError('unsupported gripper controller state')
    before = query()
    if before == target:
        # Restoration is idempotent, but still requires a confirmed state.
        return {'before': before, 'after': before, 'response_received': None}
    if before not in ('active', 'inactive'):
        raise RuntimeError(f'gripper controller is not configured: {before}')
    acknowledged = request(target)  # Exactly once; None means response timeout.
    after = query()
    if acknowledged is False or after != target:
        raise RuntimeError(f'gripper switch not confirmed: {before} -> {after}, response={acknowledged}')
    return {'before': before, 'after': after, 'response_received': acknowledged is not None}


def main(target):
    import json
    import rclpy
    from controller_manager_msgs.srv import ListControllers, SwitchController

    rclpy.init()
    node = rclpy.create_node('acceptance_gripper_switch')
    listing = node.create_client(ListControllers, '/controller_manager/list_controllers')
    switching = node.create_client(SwitchController, '/controller_manager/switch_controller')
    try:
        if not listing.wait_for_service(timeout_sec=5.) or not switching.wait_for_service(timeout_sec=5.):
            raise RuntimeError('isolated controller manager unavailable')

        def call(client, req):
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
            if not future.done():
                future.cancel()
                return None
            return future.result()

        def query():
            response = call(listing, ListControllers.Request())
            if response is None:
                raise RuntimeError('controller state readback timed out')
            matches = [c.state for c in response.controller if c.name == 'gripper_controller']
            if len(matches) != 1:
                raise RuntimeError('missing or ambiguous gripper controller')
            return matches[0]

        def request(state):
            req = SwitchController.Request()
            req.strictness = SwitchController.Request.STRICT
            req.timeout.sec = 5
            if state == 'active':
                req.activate_controllers = ['gripper_controller']
            else:
                req.deactivate_controllers = ['gripper_controller']
            response = call(switching, req)
            return None if response is None else bool(response.ok)

        result = transition(query, request, target)
        print(json.dumps(result), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
