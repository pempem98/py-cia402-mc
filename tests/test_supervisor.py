"""Supervisor behaviour with a fake pysoem master (no hardware)."""
import threading
import time

import pysoem

from ethercat_mc.config import BusConfig
from ethercat_mc.master import EtherCATMaster


class FakeSlave:
    def __init__(self, state):
        self.state = state
        self.al_status = 0
        self.name = "fake"
        self.writes = []

    def write_state(self):
        self.writes.append(self.state)


class FakeMaster:
    def __init__(self, states):
        self.slaves = [FakeSlave(s) for s in states]
        self._states = list(states)

    def read_state(self):
        for s, st in zip(self.slaves, self._states):
            s.state = st


def make(states):
    em = EtherCATMaster(BusConfig())
    em.master = FakeMaster(states)
    return em


def run_supervisor(em, seconds=0.35):
    em.start_supervisor(period_s=0.05)
    time.sleep(seconds)
    em.stop_supervisor()


def test_unreadable_state_is_not_treated_as_leaving_op():
    """0x00 means the status read got no reply; the old code sent the slave
    to PRE-OP over it and took a healthy EPOS4 off the bus."""
    em = make([pysoem.NONE_STATE])
    run_supervisor(em)
    assert em.master.slaves[0].writes == []
    assert em.op_drops == [0]


def test_real_safeop_is_recovered():
    em = make([pysoem.SAFEOP_STATE])
    run_supervisor(em)
    assert em.master.slaves[0].writes  # an OP request or a PRE-OP round trip
    assert em.op_drops[0] > 0


def test_healthy_op_slave_is_left_alone():
    em = make([pysoem.OP_STATE])
    run_supervisor(em)
    assert em.master.slaves[0].writes == []
