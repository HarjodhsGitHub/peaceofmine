"""Reject browser control connections opened by another website."""
import unittest
from unittest.mock import Mock
from urllib.parse import urlsplit
from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from test_arm_node import module


class WebOriginTest(unittest.IsolatedAsyncioTestCase):
    async def test_foreign_origin_rejected_same_origin_allowed(self):
        node = Mock()
        node.get_parameter.return_value.value = 'http://127.0.0.1:8081'
        node.connected_sockets.return_value = []
        app = module('operator_web_node').build_app(node)
        async with TestClient(TestServer(app)) as client:
            with self.assertRaises(WSServerHandshakeError) as error:
                await client.ws_connect('/ws', origin='https://other.example')
            self.assertEqual(error.exception.status, 403)
            node.connect_client.assert_not_called()
            origin = str(client.make_url('/')).rstrip('/')
            async with client.ws_connect('/ws', origin=origin):
                node.connect_client.assert_called_once()
