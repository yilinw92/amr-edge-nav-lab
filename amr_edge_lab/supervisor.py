"""Deterministic route supervisor, independent of ROS callbacks and wall clock."""
import math
TERMINAL = {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED', 'FAULT'}

class RouteSupervisor:

    def __init__(self, recovery_delay=1.0, timeout=120.0, max_retries=2, cancel_timeout=10.0, max_pauses=3):
        self.recovery_delay, self.timeout, self.max_retries, self.cancel_timeout, self.max_pauses = (recovery_delay, timeout, max_retries, cancel_timeout, max_pauses)
        self.state = 'IDLE'
        self.route = []
        self.index = 0
        self.token = 0
        self.retry = 0
        self.pauses = 0
        self.events = []
        self.healthy_since = None
        self.cancel_reason = None
        self.sent_at = 0
        self.cancel_at = 0
        self.route_id = 0

    def event(self, now, kind, **fields):
        self.events.append({'seq': len(self.events) + 1, 't': round(now, 4), 'event': kind, 'route_id': self.route_id, 'waypoint': self.index, 'token': self.token, **fields})

    def begin(self, route, now):
        if self.state not in TERMINAL or self.state == 'FAULT':
            raise RuntimeError('previous action must settle; FAULT requires inspection')
        if not route or len(route) > 100 or any((len(p) != 2 or not all((math.isfinite(v) for v in p)) for p in route)):
            raise ValueError('route needs 1..100 finite XY goals')
        self.route = list(route)
        self.index = 0
        self.retry = 0
        self.pauses = 0
        self.route_id += 1
        self.state = 'READY'
        self.cancel_reason = None
        self.healthy_since = None
        self.event(now, 'ROUTE_STARTED', goals=route)

    def cancel(self, reason, now):
        if self.state in TERMINAL:
            return False
        if self.state in {'READY', 'PAUSED'}:
            self.state = 'CANCELED' if reason == 'operator' else 'FAILED'
            self.event(now, 'ROUTE_STOPPED', reason=reason)
            return True
        if self.cancel_reason is None or reason == 'operator':
            self.cancel_reason = reason
            self.cancel_at = now
            self.state = 'CANCELING'
            self.event(now, 'CANCEL_REQUESTED', reason=reason)
        return True

    def step(self, healthy, now, unhealthy_reason='sensor_stale'):
        if healthy:
            if self.healthy_since is None:
                self.healthy_since = now
        else:
            self.healthy_since = None
            if self.state in {'WAITING_FOR_ACCEPTANCE', 'EXECUTING'}:
                self.cancel(unhealthy_reason, now)
        if self.state == 'CANCELING':
            if now - self.cancel_at > self.cancel_timeout:
                self.state = 'FAULT'
                self.event(now, 'CANCEL_RESULT_TIMEOUT')
            return None
        if self.state in {'WAITING_FOR_ACCEPTANCE', 'EXECUTING'} and now - self.sent_at > self.timeout:
            self.cancel('mission_timeout', now)
            return None
        delay = 0 if self.state == 'READY' else self.recovery_delay
        if self.state in {'READY', 'PAUSED'} and healthy and (now - self.healthy_since >= delay):
            self.token += 1
            self.sent_at = now
            self.cancel_reason = None
            self.state = 'WAITING_FOR_ACCEPTANCE'
            self.event(now, 'GOAL_SENT', goal=self.route[self.index], retry=self.retry)
            return (self.token, self.route[self.index])
        return None

    def accepted(self, token, accepted, now):
        if token != self.token or self.state not in {'WAITING_FOR_ACCEPTANCE', 'CANCELING'}:
            self.event(now, 'STALE_ACCEPTANCE_IGNORED', received=token)
            return
        self.event(now, 'GOAL_ACCEPTED' if accepted else 'GOAL_REJECTED')
        if not accepted:
            self.result(token, 'failed', now)
        elif self.cancel_reason is None:
            self.state = 'EXECUTING'

    def result(self, token, outcome, now):
        if token != self.token or self.state not in {'WAITING_FOR_ACCEPTANCE', 'EXECUTING', 'CANCELING'}:
            self.event(now, 'STALE_RESULT_IGNORED', received=token)
            return
        if outcome not in {'succeeded', 'canceled', 'failed'}:
            raise ValueError('unknown action outcome')
        self.event(now, 'GOAL_RESULT', outcome=outcome, reason=self.cancel_reason)
        if self.cancel_reason == 'operator':
            self.state = 'CANCELED'
        elif self.cancel_reason == 'mission_timeout':
            self.state = 'FAILED'
        elif outcome == 'succeeded':
            self.index += 1
            self.retry = 0
            self.state = 'SUCCEEDED' if self.index == len(self.route) else 'PAUSED'
            self.healthy_since = None
        elif self.cancel_reason in {'sensor_stale', 'navigator_unavailable', 'health_heartbeat_stale'}:
            self.pauses += 1
            self.state = 'PAUSED' if self.pauses <= self.max_pauses else 'FAILED'
            self.healthy_since = None
        else:
            self.retry += 1
            self.state = 'PAUSED' if self.retry <= self.max_retries else 'FAILED'
            self.healthy_since = None
        if self.state in TERMINAL:
            self.event(now, 'ROUTE_FINISHED', outcome=self.state)
