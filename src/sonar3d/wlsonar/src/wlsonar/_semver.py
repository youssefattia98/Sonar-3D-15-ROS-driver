def _semver_is_less_than(a: str, b: str) -> bool:
    a_parts = [int(x) for x in a.split(".")]
    b_parts = [int(x) for x in b.split(".")]
    if len(a_parts) != 3 or len(b_parts) != 3:
        raise ValueError("semver strings must have three parts")
    for a_part, b_part in zip(a_parts, b_parts):
        if a_part < b_part:
            return True
        elif a_part > b_part:
            return False
    # got here: they are equal
    return False
