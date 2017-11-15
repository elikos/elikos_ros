

STOPPED = 0
PENDING = 1
RUNNING = 2
FAILING = 3
CRASHED = 4

ALLOWED_TRANSITIONS = {
    STOPPED: [],
    PENDING: [RUNNING, FAILING],
    RUNNING: [FAILING],
    FAILING: [RUNNING],
    CRASHED: []
}


def is_allowed_transition(old_status, new_status):
    return ALLOWED_TRANSITIONS[old_status].contains(new_status)


def to_string(status):
    if status == STOPPED:
        return "STOPPED"
    if status == PENDING:
        return "PENDING"
    if status == RUNNING:
        return "RUNNING"
    if status == FAILING:
        return "FAILING"
    if status == CRASHED:
        return "CRASHED"
    raise ValueError("status is not a node status")
