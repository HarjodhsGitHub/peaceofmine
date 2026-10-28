"""Nonblocking persistence client. Device nodes validate and apply their own settings."""
import json
import time
from peaceofmine_interfaces.srv import SaveSettings
from . import configuration


class SettingsClient:
    def __init__(self, node, path):
        self.node, self.path = node, path
        self.client = node.create_client(SaveSettings, 'operator/settings/save')
        self.pending = None
        node.create_timer(.1, self.check_timeout)

    def save(self, section, value, complete):
        if self.pending:
            raise ValueError('A settings save is already pending')
        if not self.client.service_is_ready():
            raise ValueError('Settings service unavailable; nothing was saved')
        if getattr(self.node,'simulation',False) and section in ('arm','probe','arm_motion','probe_motion'):
            section += '_simulation'
        revision = configuration.load(self.path).get('revision', 0)
        request = SaveSettings.Request(section=section, value_json=json.dumps(value, allow_nan=False), expected_revision=revision)
        future = self.client.call_async(request)
        self.pending = future, time.monotonic(), complete
        future.add_done_callback(self.done)

    def done(self, future):
        if not self.pending or self.pending[0] is not future:
            return
        _, _, complete = self.pending
        self.pending = None
        try:
            response = future.result()
            complete(response.success, response.message)
        except Exception as exc:
            complete(False, str(exc))

    def check_timeout(self):
        if self.pending and time.monotonic()-self.pending[1] > 3:
            future, _, complete = self.pending
            self.pending = None
            self.client.remove_pending_request(future)
            complete(False, 'Save result unavailable; refresh settings before retrying')
