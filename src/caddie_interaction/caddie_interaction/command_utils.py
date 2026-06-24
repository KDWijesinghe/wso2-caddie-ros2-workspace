import re
from difflib import SequenceMatcher


START_ALIASES = {
    'start',
    'start mapping',
    'scan',
    'scan course',
    'map',
    'map course',
    'explore',
    'explore course',
}

STOP_MAPPING_ALIASES = {
    'stop mapping',
    'finish mapping',
    'end mapping',
    'freeze map',
    'mapping done',
    'scan stop',
}

STOP_MOTION_ALIASES = {
    'stop',
    'halt',
    'pause',
    'cancel',
    'cancel navigation',
    'emergency stop',
}

RETURN_HOME_ALIASES = {
    'return home',
    'go home',
    'back home',
    'return to tee',
    'go to tee',
    'return to start',
}

LIST_ALIASES = {
    'list',
    'list balls',
    'show balls',
    'what balls',
    'detected balls',
}

ANALYTICS_ALIASES = {
    'analyze shot',
    'shot analytics',
    'analyse shot',
    'shot analysis',
    'give shot advice',
}

FOLLOW_ALIASES = {
    'follow me',
    'follow golfer',
    'track me',
}

RETRIEVE_PREFIXES = (
    'retrieve ',
    'fetch ',
    'find ',
    'go to ',
    'navigate to ',
    'pick up ',
)

WAYPOINT_NAMES = {
    'tee': 'tee_box',
    'tee box': 'tee_box',
    'fairway': 'fairway',
    'green': 'green',
    'putting green': 'green',
    'clubhouse': 'clubhouse',
}

NUMBER_WORDS = {
    'zero': '0',
    'one': '1',
    'two': '2',
    'too': '2',
    'to': '2',
    'three': '3',
    'four': '4',
    'for': '4',
    'five': '5',
    'six': '6',
    'seven': '7',
    'eight': '8',
    'ate': '8',
    'nine': '9',
    'ten': '10',
}


def normalize_text(text: str) -> str:
    text = text.strip().lower().replace('_', ' ')
    text = re.sub(r'[^a-z0-9 ]+', ' ', text)
    return ' '.join(text.split())


def best_match(text: str, choices, cutoff: float = 0.78) -> str | None:
    best = None
    best_score = 0.0
    for choice in choices:
        score = SequenceMatcher(None, text, choice).ratio()
        if score > best_score:
            best = choice
            best_score = score
    return best if best is not None and best_score >= cutoff else None


def normalize_ball_label(text: str) -> str:
    words = normalize_text(text).split()
    if not words:
        return 'nearest'
    converted = [NUMBER_WORDS.get(word, word) for word in words]
    label = '_'.join(converted)
    if label.isdigit():
        return f'ball_{label}'
    if label.startswith('ball_'):
        return label
    if label.startswith('ball') and len(label) > 4:
        suffix = label[4:].lstrip('_')
        if suffix:
            return f'ball_{suffix}'
    return label


def canonicalize_command(text: str) -> str | None:
    cmd = normalize_text(text)
    if not cmd:
        return None

    all_exact = (
        START_ALIASES | STOP_MAPPING_ALIASES | STOP_MOTION_ALIASES |
        RETURN_HOME_ALIASES | LIST_ALIASES | ANALYTICS_ALIASES |
        FOLLOW_ALIASES
    )
    fuzzy = best_match(cmd, all_exact)
    if fuzzy is not None:
        cmd = fuzzy

    if cmd in START_ALIASES:
        return 'start_mapping'
    if cmd in STOP_MAPPING_ALIASES:
        return 'stop_mapping'
    if cmd in STOP_MOTION_ALIASES:
        return 'stop'
    if cmd in RETURN_HOME_ALIASES:
        return 'return_home'
    if cmd in LIST_ALIASES:
        return 'list_balls'
    if cmd in ANALYTICS_ALIASES:
        return 'analyze_shot'
    if cmd in FOLLOW_ALIASES:
        return 'follow_golfer'

    for waypoint_text, waypoint_id in WAYPOINT_NAMES.items():
        if cmd in {waypoint_text, f'go {waypoint_text}', f'go to {waypoint_text}', f'navigate to {waypoint_text}'}:
            return f'go_to_waypoint {waypoint_id}'

    for prefix in RETRIEVE_PREFIXES:
        if cmd.startswith(prefix):
            target = cmd[len(prefix):].strip()
            if not target or target in {'ball', 'golf ball', 'the ball', 'nearest ball'}:
                return 'retrieve_ball nearest'
            for waypoint_text, waypoint_id in WAYPOINT_NAMES.items():
                if target == waypoint_text:
                    return f'go_to_waypoint {waypoint_id}'
            target = target.replace('golf ball', 'ball').replace('the ball', 'ball')
            target = target.replace('number ', '')
            return f'retrieve_ball {normalize_ball_label(target)}'

    if 'closest ball' in cmd or 'nearest ball' in cmd:
        return 'retrieve_ball nearest'
    if 'retrieve' in cmd and 'ball' in cmd:
        return 'retrieve_ball nearest'
    if 'find' in cmd and 'ball' in cmd:
        return 'retrieve_ball nearest'

    return None
