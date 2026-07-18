# Copyright 2026 KAS-lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading

from aal.adaptation_manager_node import AdaptationManager

from aal_msgs.msg import Adaptation
from aal_msgs.srv import AdaptArchitecture

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


@pytest.fixture(scope='module', autouse=True)
def rclpy_runtime():
    if not rclpy.ok():
        rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


class MockLifecycleNode(Node):
    """Stand-in for a real lifecycle node's get_state/change_state services."""

    def __init__(self, name, current_state_id, succeed_transitions=None):
        super().__init__(name)
        self.current_state_id = current_state_id
        self.succeed_transitions = succeed_transitions
        self.received_transitions = []
        self.create_service(GetState, '/' + name + '/get_state', self._get_state_cb)
        self.create_service(ChangeState, '/' + name + '/change_state', self._change_state_cb)

    def _get_state_cb(self, request, response):
        response.current_state = State(id=self.current_state_id)
        return response

    def _change_state_cb(self, request, response):
        self.received_transitions.append(request.transition.id)
        if self.succeed_transitions is None:
            response.success = True
        else:
            response.success = request.transition.id in self.succeed_transitions
        return response


@pytest.fixture
def manager():
    node = AdaptationManager()
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def executor(manager):
    ex = MultiThreadedExecutor()
    ex.add_node(manager)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        yield ex
    finally:
        ex.shutdown()
        t.join(timeout=2.0)


def _state_goal(label):
    ids = {
        'unconfigured': State.PRIMARY_STATE_UNCONFIGURED,
        'inactive': State.PRIMARY_STATE_INACTIVE,
        'active': State.PRIMARY_STATE_ACTIVE,
        'finalized': State.PRIMARY_STATE_FINALIZED,
    }
    return State(id=ids[label], label=label)


def _add_mock_node(executor, name, current_state_id, succeed_transitions=None):
    mock = MockLifecycleNode(name, current_state_id, succeed_transitions)
    executor.add_node(mock)
    return mock


def test_create_get_state_client_normalizes_dict_key(manager):
    manager.create_get_state_client('/mock_node_gs')

    assert 'mock_node_gs' in manager.get_state_client_dict
    assert '/mock_node_gs' not in manager.get_state_client_dict
    client = manager.get_state_client_dict['mock_node_gs']
    assert client.srv_type is GetState
    assert client.srv_name == '/mock_node_gs/get_state'


def test_create_change_state_client_normalizes_dict_key(manager):
    manager.create_change_state_client('/mock_node_cs')

    assert 'mock_node_cs' in manager.change_state_client_dict
    assert '/mock_node_cs' not in manager.change_state_client_dict
    client = manager.change_state_client_dict['mock_node_cs']
    assert client.srv_type is ChangeState
    assert client.srv_name == '/mock_node_cs/change_state'


def test_execute_lc_adaptation_normalizes_leading_slash(executor, manager):
    mock = _add_mock_node(executor, 'mock_node_lc_slash', State.PRIMARY_STATE_INACTIVE)

    transition = Transition(id=Transition.TRANSITION_ACTIVATE, label='activate')
    assert manager.execute_lc_adaptation(transition, '/mock_node_lc_slash') is True
    assert mock.received_transitions == [Transition.TRANSITION_ACTIVATE]
    assert 'mock_node_lc_slash' in manager.change_state_client_dict
    assert '/mock_node_lc_slash' not in manager.change_state_client_dict


def test_desiredstate_multi_hop_configure_then_activate(executor, manager):
    mock = _add_mock_node(executor, 'mock_node_multi_hop', State.PRIMARY_STATE_UNCONFIGURED)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), 'mock_node_multi_hop') is True
    assert mock.received_transitions == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_desiredstate_multi_hop_with_leading_slash_node_name(executor, manager):
    # Regression test: the adaptation's node_name may arrive with a leading
    # slash. execute_desiredstate_adaptation must normalize it consistently
    # with the change_state client lookup performed inside
    # execute_lc_adaptation, or the two disagree on the dict key and the
    # transition call raises a KeyError instead of returning a result.
    mock = _add_mock_node(executor, 'mock_node_multi_hop_slash', State.PRIMARY_STATE_UNCONFIGURED)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), '/mock_node_multi_hop_slash') is True
    assert mock.received_transitions == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_desiredstate_slash_and_noslash_share_one_client(executor, manager):
    # The same underlying node addressed with and without a leading slash
    # must resolve to a single cached client, not a second independent one.
    mock = _add_mock_node(executor, 'mock_node_dup', State.PRIMARY_STATE_UNCONFIGURED)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), 'mock_node_dup') is True
    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), '/mock_node_dup') is True

    assert mock.received_transitions == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE,
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
    assert len(manager.get_state_client_dict) == 1
    assert len(manager.change_state_client_dict) == 1


def test_desiredstate_single_hop_deactivate(executor, manager):
    mock = _add_mock_node(executor, 'mock_node_deactivate', State.PRIMARY_STATE_ACTIVE)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('inactive'), 'mock_node_deactivate') is True
    assert mock.received_transitions == [Transition.TRANSITION_DEACTIVATE]


def test_desiredstate_same_state_is_noop(executor, manager):
    mock = _add_mock_node(executor, 'mock_node_noop', State.PRIMARY_STATE_ACTIVE)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), 'mock_node_noop') is True
    assert mock.received_transitions == []


def test_desiredstate_unsupported_path_fails(executor, manager):
    mock = _add_mock_node(executor, 'mock_node_finalized', State.PRIMARY_STATE_FINALIZED)

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), 'mock_node_finalized') is False
    assert mock.received_transitions == []


def test_desiredstate_first_transition_failure_aborts_remaining(executor, manager):
    mock = _add_mock_node(
        executor, 'mock_node_first_fails', State.PRIMARY_STATE_UNCONFIGURED,
        succeed_transitions=set())

    assert manager.execute_desiredstate_adaptation(
        _state_goal('active'), 'mock_node_first_fails') is False
    # ACTIVATE must never be attempted once CONFIGURE fails.
    assert mock.received_transitions == [Transition.TRANSITION_CONFIGURE]


def test_adaptation_requested_does_not_mask_earlier_failure(executor, manager):
    # Regression test: adaptation_results used to be re-initialized inside the
    # per-adaptation loop, so response.success only reflected the last item.
    failing_node = _add_mock_node(
        executor, 'mock_node_agg_fail', State.PRIMARY_STATE_UNCONFIGURED,
        succeed_transitions=set())
    succeeding_node = _add_mock_node(
        executor, 'mock_node_agg_ok', State.PRIMARY_STATE_ACTIVE)

    client_node = Node('adapt_architecture_test_client')
    executor.add_node(client_node)

    cli = client_node.create_client(AdaptArchitecture, '/adapt_architecture')
    assert cli.wait_for_service(timeout_sec=5.0)

    request = AdaptArchitecture.Request(adaptations=[
        Adaptation(
            adaptation_target=Adaptation.DESIREDSTATE,
            node_name='mock_node_agg_fail',
            lifecycle_state_goal=_state_goal('active')),
        Adaptation(
            adaptation_target=Adaptation.DESIREDSTATE,
            node_name='mock_node_agg_ok',
            lifecycle_state_goal=_state_goal('inactive')),
    ])

    future = cli.call_async(request)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)

    assert future.done()
    assert future.result().success is False
    assert failing_node.received_transitions == [Transition.TRANSITION_CONFIGURE]
    assert succeeding_node.received_transitions == [Transition.TRANSITION_DEACTIVATE]
