var controllers = {};
var requestAnimationFrame = window.mozRequestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.requestAnimationFrame;


function connectHandler(event) {
    addGamepad(event.gamepad);
}


function addGamepad(gamepad) {
    controllers[gamepad.index] = gamepad;

    var controllerDisplay = document.createElement("div");
    controllerDisplay.setAttribute("id", "controller" + gamepad.index);

    var title = document.createElement("h1");
    title.appendChild(document.createTextNode("gamepad: " + gamepad.id));
    controllerDisplay.appendChild(title);

    var buttonDisplay = document.createElement("div");
    buttonDisplay.className = "buttons";

    for (let i = 0; i < gamepad.buttons.length; i++) {
        var singleButton = document.createElement("span");
        singleButton.className = "button";
        singleButton.innerHTML = i; // add the button number to itself
        buttonDisplay.appendChild(singleButton);
    }

    controllerDisplay.appendChild(buttonDisplay);
    var axisDisplay = document.createElement("div");
    axisDisplay.className = "axes";

    for (let i = 0; i < gamepad.axes.length; i++) {
        singleAxis = document.createElement("span");
        singleAxis.className = "axis";
        singleAxis.innerHTML = i;
        axisDisplay.appendChild(singleAxis);
    }

    controllerDisplay.appendChild(axisDisplay);
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


function updateStatus() {
    scanGamepads();

    for (controllerId in controllers) {
        var controller = controllers[controllerId];
        var controllerDisplay = document.getElementById("controller" + controllerId);

        var buttonDisplay = controllerDisplay.getElementsByClassName("button");
        for (let i = 0; i < controller.buttons.length; i++) {
            var singleButton = buttonDisplay[i];
            singleButton.innerHTML = "button" + i + ": " + controller.buttons[i].pressed;
        }

        var axisDisplay = controllerDisplay.getElementsByClassName("axis");
        for (let i = 0; i < controller.axes.length; i++) {
            var singleAxis = axisDisplay[i];
            singleAxis.innerHTML = "axis" + i + ": " + controller.axes[i].toFixed(4);
            singleAxis.setAttribute("value", controller.axes[i]);
        }
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


var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
if (haveEvents) {
    window.addEventListener("gamepadconnected", connectHandler);
    window.addEventListener("gamepaddisconnected", disconnectHandler);
} else if (haveWebkitEvents) {
    window.addEventListener("webkitgamepadconnected", connectHandler);
    window.addEventListener("webkitgamepaddisconnected", disconnectHandler);
} else {
    setInterval(scanGamepads, 1000);
}
