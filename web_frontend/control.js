/*
var controllers = {};
var requestAnimationFrame = window.mozRequestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.requestAnimationFrame;
var webSocket;
var connectionOpen = false;
var messageDiv;
var textInput;
var hostURL;
var websocketReadyStateArray;
var connectBtn;
var sendRudderBtn;
var sendSailBtn;
var disconnectBtn;

const RUDDERMIN = 0.6804406643
const RUDDERMAX = 0.7629017234
const SAILMIN = 0.6495254636
const SAILMAX = 0.6391796470


function init() {
    messageDiv = document.getElementById("message");

    hostURL = "ws://localhost:9999/";
    websocketReadyStateArray = new Array("Connecting", "Connected", "Closing", "Closed");
}


function connect() {
    try {
        webSocket = new WebSocket(hostURL);
        messageDiv.innerHTML = "<p>Socket status: " + websocketReadyStateArray[webSocket.readyState] + "</p>";

        webSocket.onopen = function () {
            messageDiv.innerHTML = "<p>Socket status: " + websocketReadyStateArray[webSocket.readyState] + "</p>";
            connectionOpen = true
        }

        webSocket.onclose = function () {
            messageDiv.innerHTML = "<p>Socket status: " + websocketReadyStateArray[webSocket.readyState] + "</p>";
            connectionOpen = false
        }
    } catch (exception) {
        messageDiv.innerHTML += "Exception caught: " + exception;
    }
}


function connectHandler(event) {
    addGamepad(event.gamepad);
    connect();
}


function addGamepad(gamepad) {
    controllers[gamepad.index] = gamepad;

    var controllerDisplay = document.createElement("div");
    controllerDisplay.setAttribute("id", "controller" + gamepad.index);

    var title = document.createElement("h1");
    title.appendChild(document.createTextNode("Using: " + gamepad.id));
    title.setAttribute("style", "text-align: center;");
    controllerDisplay.appendChild(title);

    var boatDisplay = document.createElement("div");
    boatDisplay.className = "boat";

    var rudderDisplay = document.createElement("div");
    rudderDisplay.className = "rudder";
    rudderDisplay.innerHTML = "Rudder Angle: unknown";
    boatDisplay.appendChild(rudderDisplay);

    var sailDisplay = document.createElement("div");
    sailDisplay.className = "sail";
    sailDisplay.innerHTML = "Sail Angle: unknown";
    boatDisplay.appendChild(sailDisplay);

    controllerDisplay.appendChild(boatDisplay);
    document.getElementById("start").style.display = "none";
    document.body.appendChild(controllerDisplay);
    requestAnimationFrame(updateStatus);
}


function disconnectHandler(event) {
    removeGamepad(event.gamepad);
}


function removeGamepad(gamepad) {
    let disconnectedController = document.getElementById("controller" + gamepad.index);
    document.body.removeChild(disconnectedController);
    delete controllers[gamepad.index];
}


function padTimeWithZero(digit_string) {
    if (digit_string.length < 2) {
        return "0" + digit_string;
    } else {
        return digit_string;
    }
}


function sendJSON(messageType, controllerData) {
    try {
        currDate = new Date();
        currHour = currDate.getHours().toString();
        currHour = padTimeWithZero(currHour);
        currMinutes = currDate.getMinutes().toString();
        currMinutes = padTimeWithZero(currMinutes);
        currSeconds = currDate.getSeconds().toString();
        currSeconds = padTimeWithZero(currSeconds);
        currentTime = currHour + ":" + currMinutes + ":" + currSeconds;

        jsonObj = { time: currentTime, type: messageType, data: controllerData }
        tmpSendText = JSON.stringify(jsonObj)
        webSocket.send(tmpSendText);
    } catch (exception) {
        messageDiv.innerHTML = "<p>Send error : " + exception + "</p>"
    }
}


function transformRudderAngle(rawData) {
    var maxTurn = 15;

    if (rawData < 0) {
        return (rawData / RUDDERMIN) * maxTurn;
    } else if (rawData > 0) {
        return (rawData / RUDDERMAX) * maxTurn;
    }

    return rawData;
}


function transformSailAngle(rawData) {
    var maxTurn = 15;

    if (rawData < 0) {
        return (rawData / SAILMIN) * maxTurn;
    } else if (rawData > 0) {
        return (rawData / SAILMAX) * maxTurn;
    }

    return rawData;
}


function updateStatus() {
    scanGamepads();

    if (connectionOpen) {
        // extents of axes do not match up with -1.0 to 1.0, so must be corrected
        var rudderAngle = transformRudderAngle(controllers[0].axes[0].toFixed(10));
        var sailAngle = transformSailAngle(controllers[0].axes[2].toFixed(10));

        sendJSON("rudder", rudderAngle)
        sendJSON("sail", sailAngle)

        var controllerDisplay = document.getElementById("controller0");
        var rudderDisplay = controllerDisplay.getElementsByClassName("rudder")[0];
        var sailDisplay = controllerDisplay.getElementsByClassName("sail")[0];
        rudderDisplay.innerHTML = "Rudder Angle: " + rudderAngle;
        sailDisplay.innerHTML = "Sail Angle: " + sailAngle;
    }

    requestAnimationFrame(updateStatus);
}


function scanGamepads() {
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && (gamepads[i].index in controllers)) {
            controllers[gamepads[i].index] = gamepads[i];
        }
    }
}



function disconnect() {
    webSocket.close();
}


var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
connect();
if (haveEvents) {
    window.addEventListener("gamepadconnected", connectHandler);
    window.addEventListener("gamepaddisconnected", disconnectHandler);
} else if (haveWebkitEvents) {
    window.addEventListener("webkitgamepadconnected", connectHandler);
    window.addEventListener("webkitgamepaddisconnected", disconnectHandler);
} else {
    setInterval(scanGamepads, 1000);
}
*/const exampleSocket = new WebSocket(
    "ws://localhost:9999",
    "protocolOne",
  );

ws.on("connection", (ws) => {
  console.log("I'm server")
  let msg = "Connection Established!. M2C!";
  ws.send(JSON.stringify(msg));
});
