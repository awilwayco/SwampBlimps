// Connect to SocketIO server
const socket = io();

// Receive Connection
socket.on('connect', () => {
    // Client IP Address
    console.log('Client connected with IP:', client_ip);

    // Load current backend state
    socket.emit('update_frontend');
});

socket.on('update', (blimp_dict) => {

    // Debugging
    //console.log(blimp_dict)

    update_basestation(blimp_dict);
});

socket.on('remove', (blimp_id) => {

  // Debugging
  //console.log(blimp_id)

  remove_blimp(blimp_id);
});

socket.on('reload', () => {
  window.location.reload();
})

socket.on('start', () => {
  window.location.reload();
  console.log('Starting up Basestation');
});

socket.on('kill_backend', () => {
  socket.emit('kill_basestation');
});

socket.on('kill', () => {
  window.location.reload();
  console.log('Shutting down Basestation');
});

socket.on('update_controlled')

// Catching Blimps before Attack Blimps
var blimpOrder = ["BurnCreamBlimp", "SillyAhBlimp", "TurboBlimp", "GameChamberBlimp", "FiveGuysBlimp", "SuperBeefBlimp", "Catch1", "Catch2", 'Yoshi', 'Luigi', 'Geoph', 'ThisGuy', "Attack1", "Attack2"];

// Unordered List of Blimp Names

var blimpList = [];

// Ordered Maps

var sortedNameRows = {};

var sortedStateRows = {};

var sortedLinkRows = {};

var blimpsTableBody = document.getElementById('blimpsTableBody');

var statesTableBody = document.getElementById('statesTableBody');

var streamTableBody = document.getElementById('streamTableBody');

var goalButtonsContainer = document.querySelector('.goalButtonsContainer');

var targetButtonsContainer = document.querySelector('.targetButtonsContainer');

var leftStickX = 0;
var leftStickY = 0;
var rightStickX = 0;
var rightStickY = 0;

function update_basestation(blimp_dict) {
  // Get data from blimp dictionary
  let blimp_id = blimp_dict["blimp_id"];

  // Debugging
  //console.log(blimp_dict);

  // Get goal color from the data
  let goal_color;
  if (blimp_dict["goal_color"] === 0) {
      goal_color = 'orange';
  } else {
      goal_color = 'yellow';
  }

  // Get the goal color button for the specific blimp
  let goal_color_button = document.getElementById(`goal_color_button_${blimp_id}`);

  // Number of targets
  let target_num;

  // Target Buttons
  let target_color_1_button;
  let target_color_2_button;

  // Target Color (Attack Blimps)
  let target_color;

  // State Machine
  let state;

  // Get blimp type from the data
  let blimp_type = blimp_dict["blimp_type"];
  if (blimp_type === 0) {
    target_num = 2;
    target_color_1_button = document.getElementById(`target_color_1_button_${blimp_id}`);
    target_color_2_button = document.getElementById(`target_color_2_button_${blimp_id}`);
  }
  else {
    target_num = 1;
    target_color_1_button = document.getElementById(`target_color_1_button_${blimp_id}`);
    if (blimp_dict["target_color"] === 0) {
      target_color = 'blue';
    } else {
      target_color = 'red';
    }    
  }

  // Only proceed if the name is not in the table
  if (!(blimpList.includes(blimp_id))) {
    // Create a new row and cell for the blimp name
    var newRow = document.createElement('h3');
    var newCell = document.createElement('h3');
    newCell.textContent = blimp_dict["blimp_name"];
    newRow.appendChild(newCell);
    blimpsTableBody.appendChild(newRow);

    // Store the row in the blimpList
    blimpList.push(blimp_id);

    // Store the state in sortedNameRows using blimp_id as the key
    sortedNameRows[blimp_id] = newRow;

    // Sort the name rows based on the blimp names and their states
    var sortedRows = Object.keys(sortedNameRows).sort(function (a, b) {
        return blimpOrder.indexOf(a) - blimpOrder.indexOf(b);
    });

    // Clear the existing content of blimpsTableBody
    blimpsTableBody.innerHTML = '';

    // Append the sorted rows to blimpsTableBody
    sortedRows.forEach(function (rowKey) {
        blimpsTableBody.appendChild(sortedNameRows[rowKey]);
    });

    // State Machine
    state = get_state(blimp_dict['state_machine']);

    // Clear the statesTableBody
    statesTableBody.innerHTML = '';

    // Create a new row and cell for the blimp state
    var newRow = document.createElement('h3');
    var newCell = document.createElement('h3');
    newCell.textContent = state; // Get the state using blimp_id
    newRow.appendChild(newCell);
    statesTableBody.appendChild(newRow);

    // Store the state in sortedStateRows using blimp_id as the key
    sortedStateRows[blimp_id] = newRow;

    // Sort the state rows based on the blimp names and their states
    var sortedRows = Object.keys(sortedStateRows).sort(function (a, b) {
        return blimpOrder.indexOf(a) - blimpOrder.indexOf(b);
    });

    // Clear the existing content of statesTableBody
    statesTableBody.innerHTML = '';

    // Append the sorted rows to statesTableBody
    sortedRows.forEach(function (rowKey) {
        statesTableBody.appendChild(sortedStateRows[rowKey]);
    });

    if (target_num === 2) {
      update_target_button_colors(blimp_dict, 'green', target_color_1_button, 'purple', target_color_2_button);
      update_goal_button_color(blimp_dict, goal_color, goal_color_button);
    }
    // Target number is 1 (Attack Blimps)
    else {
      update_target_button_color(blimp_dict, target_color, target_color_1_button);
    }
    // Sort the target rows based on your desired order
    sortedTargetRows = Array.from(targetButtonsContainer.querySelectorAll('[blimp_id]'));
    sortedTargetRows.sort(function(a, b) {
        const blimpNameA = a.getAttribute('blimp_id');
        const blimpNameB = b.getAttribute('blimp_id');
        return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
    });
    
    targetButtonsContainer.innerHTML = ''; // Clear the container
    sortedTargetRows.forEach(function(row) {
        targetButtonsContainer.appendChild(row);
    });

    // Clear the existing content of streamTableBody
    streamTableBody.innerHTML = '';
    
    if (blimp_dict['blimp_type'] === 0) {
      // Create hyperlink on basestation
      var newRow = document.createElement('h3');
      var newCell = document.createElement('h3');
      var hyperlink = document.createElement('a');

      // Set the hyperlink attributes
      hyperlink.href = "/" + blimp_id;
      hyperlink.target = "_blank";  // This will open the link in a new tab/window
      hyperlink.textContent = "View Stream";
      hyperlink.setAttribute("blimp_id", blimp_id);

      // Append the hyperlink to the newCell and then to the newRow
      newCell.appendChild(hyperlink);
      newRow.appendChild(newCell);
      streamTableBody.appendChild(newRow);

      // Store the link in sortedLinkRows using blimp_id as the key
      sortedLinkRows[blimp_id] = newRow;

    }

    // Sort the link rows based on the blimp names
    var sortedRows = Object.keys(sortedLinkRows).sort(function (a, b) {
      return blimpOrder.indexOf(a) - blimpOrder.indexOf(b);
    });

    // Append the sorted rows to streamTableBody
    sortedRows.forEach(function (rowKey) {
      streamTableBody.appendChild(sortedLinkRows[rowKey]);
    });

  }
  else {
    // Set goal color of the button for all clients to see
    if (blimp_type === 0) {
      if (goal_color_button) {
        goal_color_button.style.backgroundColor = goal_color;
      }
    }
    else {
      if (goal_color_button) {
        goal_color_button.style.backgroundColor = 'white';
        goal_color_button.style.border = "1px solid white";
      }
    }
    // Set target color of the button for all clients to see
    if (target_color_1_button) {
      target_color_1_button.style.backgroundColor = target_color;
    }

    // Update the state row's textContent for all clients to see
    state = get_state(blimp_dict['state_machine']);
    if (sortedStateRows[blimp_id]) {
      sortedStateRows[blimp_id].textContent = state;
    }

    //Update controller connection
    if (blimp_dict['selected']) {
      sortedNameRows[blimp_id].style.color = 'blue';
      sortedStateRows[blimp_id].style.color = 'blue';
    } else {
      sortedNameRows[blimp_id].style.color = 'black';  
      sortedStateRows[blimp_id].style.color = 'black';
    }

    //Update autonomous state
    if (blimp_dict["auto"]) {
      sortedNameRows[blimp_id].textContent = blimp_dict["blimp_name"] + ' - A';
    } else {
      sortedNameRows[blimp_id].textContent = blimp_dict["blimp_name"]
    }
  }
}

function update_target_button_color(blimp_dict, target_color, target_color_button) {
    // Get data from blimp dictionary
    let blimp_id = blimp_dict["blimp_id"];
    
    // Target Color Button does not exist
    if (!target_color_button) {
        // Create new button
        target_color_button = document.createElement('button');
        target_color_button.id = `target_color_1_button_${blimp_id}`;
        target_color_button.style.backgroundColor = target_color;
        
        // Attach the click event listener to the button
        target_color_button.addEventListener('click', (event) => {
              
              // Send the data to the backend to update over ROS
              socket.emit('update_all_target_colors');
        });
        
        // Create a div element that will be used to wrap the button and force it to a new line
        const buttonWrapper = document.createElement('div');
        buttonWrapper.appendChild(target_color_button);
        // Store the blimp name
        buttonWrapper.setAttribute('blimp_id', blimp_id);
        
        // Center the button horizontally
        buttonWrapper.style.display = 'flex';
        buttonWrapper.style.justifyContent = 'center';

        targetButtonsContainer.appendChild(buttonWrapper);
    }
}

// Function Currently Not Used; Target Colors for Attack Blimps Hard Set from ML/on Teensy //
function update_target_button_colors(blimp_dict, target_color_1, target_color_1_button, target_color_2, target_color_2_button) {
  // Get data from blimp dictionary
  let blimp_id = blimp_dict["blimp_id"];

  // Create a row wrapper for the target color buttons
  let targetRowWrapper = document.createElement('div');
  targetRowWrapper.classList.add('target-row'); // You can style this class for better alignment

  // Target Color Button 1 does not exist
  if (!target_color_1_button) {
      target_color_1_button = document.createElement('button');
      target_color_1_button.id = `target_color_1_button_${blimp_id}`;
      target_color_1_button.style.backgroundColor = target_color_1;

      targetRowWrapper.appendChild(target_color_1_button);
  }

  // Target Color Button 2 does not exist
  if (!target_color_2_button) {
      target_color_2_button = document.createElement('button');
      target_color_2_button.id = `target_color_2_button_${blimp_id}`;
      target_color_2_button.style.backgroundColor = target_color_2;

      targetRowWrapper.appendChild(target_color_2_button);
  }
  targetRowWrapper.setAttribute('blimp_id', blimp_id); // Store the blimp name
  targetButtonsContainer.appendChild(targetRowWrapper);
}

function update_goal_button_color(blimp_dict, goal_color, goal_color_button) {
  // Get data from blimp dictionary
  let blimp_id = blimp_dict["blimp_id"];
  
  // Goal Color Button does not exist
  if (!goal_color_button) {
      // Create new button
      goal_color_button = document.createElement('button');
      goal_color_button.id = `goal_color_button_${blimp_id}`;
      goal_color_button.style.backgroundColor = goal_color;
      
      // Attach the click event listener to the button
      goal_color_button.addEventListener('click', (event) => {

          // Send the data to the backend to update over ROS
          socket.emit('update_all_goal_colors');
        // }
      });

      // Create a div element that will be used to wrap the button and force it to a new line
      const buttonWrapper = document.createElement('div');
      buttonWrapper.appendChild(goal_color_button);
      buttonWrapper.setAttribute('blimp_id', blimp_id); // Store the blimp name

      goalButtonsContainer.appendChild(buttonWrapper);

      // Sort the goal rows based on your desired order
      sortedGoalRows = Array.from(goalButtonsContainer.querySelectorAll('[blimp_id]'));
      sortedGoalRows.sort(function(a, b) {
          const blimpNameA = a.getAttribute('blimp_id');
          const blimpNameB = b.getAttribute('blimp_id');
          return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
      });

      goalButtonsContainer.innerHTML = ''; // Clear the container
      sortedGoalRows.forEach(function(row) {
          goalButtonsContainer.appendChild(row);
      });
    }
}

// Function Not Used; Has Error; Not Needed Currently //
function update_empty_button_color(blimp_dict, goal_color, goal_color_button, goalButtonsContainer) {
  // Get data from blimp dictionary
  let blimp_id = blimp_dict["blimp_id"];
  
  // Goal Color Button does not exist
  if (!goal_color_button) {
      // Create new button
      goal_color_button = document.createElement('button');
      goal_color_button.id = `goal_color_button_${blimp_id}`;
      goal_color_button.style.backgroundColor = goal_color;

      // Create a div element that will be used to wrap the button and force it to a new line
      const buttonWrapper = document.createElement('div');
      buttonWrapper.appendChild(goal_color_button);
      buttonWrapper.setAttribute('blimp_id', blimp_id); // Store the blimp name

      // Error Currently !!!
      goalButtonsContainer.appendChild(buttonWrapper);

      // Sort the goal rows based on your desired order
      sortedGoalRows = Array.from(goalButtonsContainer.querySelectorAll('[blimp_id]'));
      sortedGoalRows.sort(function(a, b) {
          const blimpNameA = a.getAttribute('blimp_id');
          const blimpNameB = b.getAttribute('blimp_id');
          return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
      });

      goalButtonsContainer.innerHTML = ''; // Clear the container
      sortedGoalRows.forEach(function(row) {
          goalButtonsContainer.appendChild(row);
      });
    }
}

function remove_blimp(blimp_id) {
  // Check if the blimp name is in the list
  if (blimpList.includes(blimp_id)) {
    // Remove the id row from the map if the key matches blimp_id
    Object.entries(sortedNameRows).forEach(([key, value]) => {
      if (key === blimp_id) {
        value.textContent = ''; // Set the value to ''
      }
    });

    blimpList = blimpList.filter(item => item !== blimp_id);

    // Remove the state row from the map if the key matches blimp_id
    Object.entries(sortedStateRows).forEach(([key, value]) => {
      if (key === blimp_id) {
        value.textContent = ''; // Set the value to ''
      }
    });

    let goal_color_button = document.getElementById(`goal_color_button_${blimp_id}`);
    if (goal_color_button) {
      goal_color_button.remove();
    }

    let target_color_1_button = document.getElementById(`target_color_1_button_${blimp_id}`);
    let target_color_2_button = document.getElementById(`target_color_2_button_${blimp_id}`);
    if (target_color_1_button) {
      target_color_1_button.remove();
    }
    if (target_color_2_button) {
      target_color_2_button.remove();
    }

    // Remove the link row from the map if the key matches blimp_id
    Object.entries(sortedLinkRows).forEach(([key, value]) => {
      if (key === blimp_id) {
        value.textContent = ''; // Set the value to ''
      }
    });
  }
}

function get_state(number) {
  let state;

  if (number === 0) {
      state = "searching"; // Default Value
  } else if (number === 1) {
      state = "approach";
  } else if (number === 2) {
      state = "catching";
  } else if (number === 3) {
      state = "caught";
  } else if (number === 4) {
      state = "goalSearch";
  } else if (number === 5) {
      state = "approachGoal";
  } else if (number === 6) {
      state = "scoringStart";
  } else if (number === 7) {
      state = "shooting";
  } else if (number === 8) {
      state = "scored";
  } else {
      state = "error"; // Error, should not happen
  }

  return state;
}

// Controller 1 UI Dots
var dot1 = document.getElementById('dot1');
var dot2 = document.getElementById('dot2');

// Toggler
let toggler = document.querySelector(".toggler");

window.addEventListener("click", event => {
  if(event.target.className == "toggler" || event.target.className == "toggle") {
    document.body.classList.toggle("show-nav");
  } else if (event.target.className == "overlay") {
    document.body.classList.remove("show-nav");
  }
  // Change Toggler Icon
  if(document.body.className == "show-nav") {
    toggler.innerHTML = "&laquo";
  } else {
    toggler.innerHTML = "&raquo";
  }
});

socket.on('view_stream', (selected_blimp_id) => {

  // Debugging
  //console.log(selected_blimp_id);

  var allLinks = document.getElementsByTagName("a");

  // Loop through each anchor element and extract the href attribute
  for (var i = 0; i < allLinks.length; i++) {
      var link = allLinks[i];
      let blimp_id = link.getAttribute("blimp_id");
      if (blimp_id === selected_blimp_id) {
        link.click();
      }
  }
});

socket.on('motor_commands', (controller_cmd) => {

  // Debugging
  //console.log(controller_cmd);

  moveDots(controller_cmd);
});

function moveDots(controller_cmd) {
  // Gamepad.axes array contains the values for all axes of the controller.
  // It's typical for the left stick to use axes 0 (X) and 1 (Y)
  // and for the right stick to use axes 2 (X) and 3 (Y).
  leftStickX = controller_cmd[0];
  leftStickY = controller_cmd[1];
  rightStickX = controller_cmd[2];
  rightStickY = controller_cmd[3];

  // Define a dead zone threshold (adjust as needed)
  const deadZero = 0.1;
  const deadOne = 0.01;

  // Apply tolerance by checking if values are within the dead zone
  leftStickX = Math.abs(leftStickX) < deadZero ? 0 : leftStickX;
  leftStickY = Math.abs(leftStickY) < deadZero ? 0 : leftStickY;
  rightStickX = Math.abs(rightStickX) < deadZero ? 0 : rightStickX;
  rightStickY = Math.abs(rightStickY) < deadZero ? 0 : rightStickY;

  leftStickX = leftStickX > 1 - deadOne ? 1 : leftStickX;
  leftStickY = leftStickY > 1 - deadOne ? 1 : leftStickY;
  rightStickX = rightStickX > 1 - deadOne ? 1 : rightStickX;
  rightStickY = rightStickY > 1 - deadOne ? 1 : rightStickY;

  leftStickX = leftStickX < -1 + deadOne ? -1 : leftStickX;
  leftStickY = leftStickY < -1 + deadOne ? -1 : leftStickY;
  rightStickX = rightStickX < -1 + deadOne ? -1 : rightStickX;
  rightStickY = rightStickY < -1 + deadOne ? -1 : rightStickY;

  leftStickX = leftStickX.toFixed(2);
  leftStickY = leftStickY.toFixed(2);
  rightStickX = rightStickX.toFixed(2);
  rightStickY = rightStickY.toFixed(2);

  // Multiply by -1 to Invert the Y-Axes
  // leftStickY = leftStickY * -1;
  // rightStickY = rightStickY * -1;
  rightStickX = rightStickX * -1;

  // Apply the joystick positions to the dot positions
  // The joystick returns a value between -1 and 1.
  // We shift this range to be 0 - 100% for use with the CSS styles.
  dot3.style.left = `${50 + leftStickX * 45}%`;
  dot3.style.top = `${50 + leftStickY * -45}%`;
  dot4.style.left = `${50 + rightStickX * 45}%`;
  dot4.style.top = `${50 + rightStickY * -45}%`;

  // Debugging
  //console.log("Left Stick X: ", 50 + leftStickX * 45);
}

// function requestFullScreen(element) {
//   // Supports most browsers and their versions.
//   var requestMethod = element.requestFullScreen || element.webkitRequestFullScreen || element.mozRequestFullScreen || element.msRequestFullScreen;

//   if (requestMethod) { // Native full screen.
//       requestMethod.call(element);
//   } else if (typeof window.ActiveXObject !== "undefined") { // Older IE.
//       var wscript = new ActiveXObject("WScript.Shell");
//       if (wscript !== null) {
//           wscript.SendKeys("{F11}");
//       }
//   }
// }

// // Get the parent container element by its class name
// var containerElement = document.querySelector('.image_container');

// // Find the link element within the container
// var linkElement = containerElement.querySelector('a');

// // Add a click event listener to the link
// linkElement.addEventListener('click', function(event) {
//   // Your custom logic here, before preventing the default behavior
//   console.log('Link clicked!');

//   // You can perform actions like changing styles, showing a message, etc.
//   console.log("Hello");
//   var elem = document.body; // Make the body go full screen.
//   requestFullScreen(elem);

//   // Prevent the default behavior of the link (e.g., navigating to a new page)
//   event.preventDefault();

//   // Your custom logic here, after preventing the default behavior
//   console.log('Prevented navigation!');

//   // You can also perform other actions or navigate to a different URL programmatically
//   //window.location.href = 'https://fakeupdate.net/xp/';
// });
let count = 0;

// Get the parent container element by its class name
var containerElement = document.querySelector('.image_container');

// Find the link element within the container
var linkElement = containerElement.querySelector('a');

// Add a click event listener to the link
linkElement.addEventListener('click', function(event) {

    // Prevent the default behavior of the link (e.g., navigating to a new page)
    if (count === 0) {
      event.preventDefault();
      toggleFullScreen();
      count = 1;
    }
    else {
      toggleFullScreen();
    }
});

function toggleFullScreen() {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen();
  } else if (document.exitFullscreen) {
    document.exitFullscreen();
  }
}