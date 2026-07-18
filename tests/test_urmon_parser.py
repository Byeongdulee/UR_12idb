import importlib.util
import struct
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "urxe" / "urmon_parser.py"
SPEC = importlib.util.spec_from_file_location("urmon_parser", MODULE_PATH)
urmon_parser = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(urmon_parser)
ParserUtils = urmon_parser.ParserUtils


def test_additional_info_parses_polyscope_5_26_layout():
    parser = ParserUtils()
    parser.version = (5, 26)
    payload = struct.pack("!iB", 11, 8) + struct.pack("!B??B", 1, False, True, 2)

    parsed = parser._get_data(
        payload,
        "!iBB??B",
        ("size", "type", "tpButtonState", "isManualFreedriveAllowed", "isFreedriveIOPressed", "tpType"),
    )

    assert parsed["tpButtonState"] == 1
    assert parsed["isManualFreedriveAllowed"] is False
    assert parsed["isFreedriveIOPressed"] is True
    assert parsed["tpType"] == 2


def test_internal_packet_types_are_ignored():
    parser = ParserUtils()
    payload = struct.pack("!iB", 6, 15) + b"\x00"

    parsed = parser.parse(payload)

    assert parsed == {}


def test_text_message_parses_message_body():
    parser = ParserUtils()
    message_text = b"hello from robot"
    payload = struct.pack("!I B Q b b", 15 + len(message_text), 20, 1234, 0, 0) + message_text

    parsed = parser.parse(payload)

    assert parsed["messageText"]["messageText"] == message_text


def test_runtime_exception_message_parses_details():
    parser = ParserUtils()
    message_text = b"script error"
    payload = struct.pack("!I B Q b b I I", 23 + len(message_text), 20, 4321, 0, 10, 7, 3) + message_text

    parsed = parser.parse(payload)

    assert parsed["runtimeExceptionMessage"]["scriptLineNumber"] == 7
    assert parsed["runtimeExceptionMessage"]["scriptColumnNumber"] == 3
    assert parsed["runtimeExceptionMessage"]["runtimeExceptionTextMessage"] == message_text


def test_label_message_parses_text_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    message_text = b"label text"
    payload = struct.pack("!I B Q b b i", 19 + len(message_text), 20, 4321, 0, 1, 42) + message_text

    parsed = parser.parse(payload)

    assert parsed["labelMessage"]["id"] == 42
    assert parsed["labelMessage"]["messageText"] == message_text


def test_popup_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    title = b"Popup title"
    message = b"Popup body"
    title_size = len(title)
    payload = struct.pack("!I B Q b b I B ? ? ? H", 25 + title_size + len(message), 20, 4321, 0, 2, 7, 1, False, True, True, title_size) + title + message

    parsed = parser.parse(payload)

    assert parsed["popupMessage"]["id"] == 7
    assert parsed["popupMessage"]["type"] == 1
    assert parsed["popupMessage"]["warning"] is False
    assert parsed["popupMessage"]["error"] is True
    assert parsed["popupMessage"]["blocking"] is True
    assert parsed["popupMessage"]["messageTitle"] == title
    assert parsed["popupMessage"]["messageText"] == message


def test_key_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    title = b"Key title"
    message = b"Key body"
    payload = struct.pack("!I B Q b b I I H", 25 + len(title) + len(message), 20, 4321, 0, 7, 11, 3, len(title)) + title + message

    parsed = parser.parse(payload)

    assert parsed["keyMessage"]["code"] == 11
    assert parsed["keyMessage"]["argument"] == 3
    assert parsed["keyMessage"]["messageTitle"] == title
    assert parsed["keyMessage"]["messageText"] == message


def test_safety_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    payload = struct.pack("!I B Q b b I I B I I", 32, 20, 4321, 0, 5, 11, 3, 2, 7, 9)

    parsed = parser.parse(payload)

    assert parsed["SafetyMessage"]["code"] == 11
    assert parsed["SafetyMessage"]["argument"] == 3
    assert parsed["SafetyMessage"]["Modetype"] == 2
    assert parsed["SafetyMessage"]["Datatype"] == 7
    assert parsed["SafetyMessage"]["Data"] == 9


def test_var_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    title = b"Var title"
    message = b"Var body"
    payload = struct.pack("!I B Q b b I I H", 25 + len(title) + len(message), 20, 4321, 0, 8, 11, 3, len(title)) + title + message

    parsed = parser.parse(payload)

    assert parsed["varMessage"]["code"] == 11
    assert parsed["varMessage"]["argument"] == 3
    assert parsed["varMessage"]["messageTitle"] == title
    assert parsed["varMessage"]["messageText"] == message


def test_robot_comm_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    message_text = b"comm message"
    payload = struct.pack("!I B Q b b", 15 + len(message_text), 20, 4321, 0, 6) + message_text

    parsed = parser.parse(payload)

    assert parsed["robotCommMessage"]["messageText"] == message_text


def test_version_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    project_name = b"project"
    payload = struct.pack("!I B Q b b B", 18 + len(project_name), 20, 4321, 0, 3, len(project_name)) + project_name + struct.pack("!B B B I I", 5, 26, 0, 42, 7)

    parsed = parser.parse(payload)

    assert parsed["VersionMessage"]["projectName"] == project_name
    assert parsed["VersionMessage"]["majorVersion"] == 5
    assert parsed["VersionMessage"]["minorVersion"] == 26
    assert parsed["VersionMessage"]["buildNumber"] == b''
    assert "buildDate" not in parsed["VersionMessage"]


def test_global_variables_setup_message_parses_details_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    variable_names = b"var1"
    payload = struct.pack("!I B Q b b", 19 + len(variable_names), 25, 4321, 0, 0) + struct.pack("!I", 0) + variable_names

    parsed = parser.parse(payload)

    assert parsed["globalVariablesSetupMessage"]["startIndex"] == 0
    assert parsed["globalVariablesSetupMessage"]["variableNames"] == b"\x00" + variable_names


def test_packet_type_24_is_tolerated_for_newer_versions():
    parser = ParserUtils()
    parser.version = (5, 8)
    payload = struct.pack("!I B", 6, 24) + b"\x00"

    parsed = parser.parse(payload)

    assert parsed == {}
