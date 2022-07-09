"""Simple GUI to launch a rocket"""

import PySimpleGUI as sg

import launcher
from krpcutils import KrpcUtilities
from circularise import Circularise
from features import Features
from logger import Logger
from connection import Connection


sg.theme("DarkAmber")

layout = [
    [sg.Text("Features")],
    [
        sg.Checkbox("Full Burn", default=False, key="FULL_BURN"),
        sg.Checkbox("Use SAS", default=True, key="USE_SAS"),
        sg.Checkbox("Perform Ascent Roll", default=True, key="ASCENT_ROLL"),
        sg.Checkbox("Show Log in KSP", default=False, key="SHOW_LOG"),
    ],
    [
        sg.Checkbox("Test At", default=False, key="TEST_AT", enable_events=True),
        sg.InputText(key="-TEST_AT-", disabled=True),
    ],
    [
        sg.Checkbox("At LAN", default=False, key="AT_LAN", enable_events=True),
        sg.InputText(key="-AT_LAN-", disabled=True),
    ],
    [
        sg.Checkbox(
            "Inclination", default=False, key="TARGET_INCLINATION", enable_events=True
        ),
        sg.InputText(key="-TARGET_INCLINATION-", disabled=True),
    ],
    [
        sg.Checkbox(
            "Altitude", default=True, key="TARGET_ALTITUDE", enable_events=True
        ),
        sg.InputText(key="-TARGET_ALTITUDE-", default_text="100000"),
    ],
    [sg.Button(button_text="Launch!"), sg.Cancel()],
]

window = sg.Window("First Window", layout=layout)

# Create object instances
my_logger = Logger().get()
my_connection = Connection(logger=my_logger, name="Launch into orbit")
my_utils = KrpcUtilities(logger=my_logger, connection=my_connection)
my_circularise = Circularise(logger=my_logger, connection=my_connection, utils=my_utils)

while True:
    event, values = window.read(timeout=500, timeout_key="-TO-")

    if event in (None, "Cancel"):
        break

    if event in ["TEST_AT", "AT_LAN", "TARGET_INCLINATION", "TARGET_ALTITUDE"]:
        el = window["-" + event + "-"]
        el.Update(disabled=not values[event])

    if event in ("Launch!"):
        features = Features()
        for f in ["FULL_BURN", "USE_SAS", "ASCENT_ROLL", "SHOW_LOG"]:
            features.set(f, values[f])
        for f in ["TEST_AT", "AT_LAN", "TARGET_INCLINATION", "TARGET_ALTITUDE"]:
            if values[f]:
                v = values["-" + f + "-"]
                if v != "":
                    features.set(f, float(v))

        launcher.launch_rocket(
            features=features,
            logger=my_logger,
            utils=my_utils,
            circularise=my_circularise,
        )


window.close()
