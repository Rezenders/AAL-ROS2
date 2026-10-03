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

"""Focused AAL failure handling tests without starting ROS nodes."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from aal.adaptation_manager_node import AdaptationManager

import pytest


def test_batch_stops_at_first_rejection():
    """Do not let a later success mask an earlier failure."""
    node = SimpleNamespace(
        _execute_checked=Mock(side_effect=[True, False, True]))
    response = AdaptationManager.adaptation_requested(
        node, SimpleNamespace(adaptations=['a', 'b', 'c']), SimpleNamespace())
    assert response.success is False
    assert node._execute_checked.call_count == 2


def test_empty_batch_succeeds():
    """An empty plan is a successful no-op."""
    node = SimpleNamespace(_execute_checked=Mock())
    response = AdaptationManager.adaptation_requested(
        node, SimpleNamespace(adaptations=[]), SimpleNamespace())
    assert response.success is True
    node._execute_checked.assert_not_called()


@pytest.mark.parametrize(
    'error', [TimeoutError('timeout'), AttributeError('logger')])
def test_checked_execution_logs_errors(error):
    """Operational timeouts return failure; programming errors propagate."""
    logger = Mock()
    node = SimpleNamespace(
        get_logger=lambda: logger, print_adaptation=Mock(),
        execute_adaptation=Mock(side_effect=error))
    adaptation = SimpleNamespace(node_name='search', adaptation_target=1)
    if isinstance(error, TimeoutError):
        assert not AdaptationManager._execute_checked(node, adaptation)
        logger.error.assert_called_once()
    else:
        with pytest.raises(AttributeError):
            AdaptationManager._execute_checked(node, adaptation)
        assert 'Traceback' in logger.fatal.call_args.args[0]
        assert 'search' in logger.fatal.call_args.args[0]


@pytest.mark.parametrize(
    'mode', ['unavailable', 'timeout', 'exception', 'success'])
def test_service_waits_are_bounded(mode):
    """Handle unavailable services, missing responses and future exceptions."""
    future = Future()
    if mode == 'exception':
        future.set_exception(ValueError('broken response'))
    elif mode == 'success':
        future.set_result('response')
    client = Mock(srv_name='/test')
    client.wait_for_service.return_value = mode != 'unavailable'
    client.call_async.return_value = future
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=0.001))
    if mode == 'success':
        result = AdaptationManager._call_service(node, client, object())
        assert result == 'response'
    else:
        error = ValueError if mode == 'exception' else TimeoutError
        with pytest.raises(error):
            AdaptationManager._call_service(node, client, object())
    if mode == 'timeout':
        assert future.cancelled()
