'use strict';
// CLASSES
class SensorData {
  constructor() {
    this.orientation = {
      alpha: 0,
      beta: 0,
      gamma: 0
    };
    this.initialOrientation = {
      alpha: 0,
      beta: 0,
      gamma: 0
    };
    this.accelerometer = {
      x: 0,
      y: 0,
      z: 0
    };
    this.gyroscope = {
      x: 0,
      y: 0,
      z: 0
    };
  }

  setInitialOrientation(alpha, beta, gamma) {
    this.initialOrientation.alpha = alpha;
    this.initialOrientation.beta = beta;
    this.initialOrientation.gamma = gamma;
  }

  updateOrientation(alpha, beta, gamma) {
    this.orientation.alpha = alpha - this.initialOrientation.alpha;
    this.orientation.beta = beta - this.initialOrientation.beta;
    this.orientation.gamma = gamma - this.initialOrientation.gamma;
  }

  updateAccelerometer(x, y, z) {
    this.accelerometer.x = x;
    this.accelerometer.y = y;
    this.accelerometer.z = z;
  }

  updateGyroscope(x, y, z) {
    this.gyroscope.x = x;
    this.gyroscope.y = y;
    this.gyroscope.z = z;
  }

  getAllData() {
    return {
      orientation: this.orientation,
      accelerometer: this.accelerometer,
      gyroscope: this.gyroscope
    };
  }

  getOrientation() {
    return { ...this.orientation };
  }

  getAccelerometer() {
    return { ...this.accelerometer };
  }

  getGyroscope() {
    return { ...this.gyroscope };
  }
}

// Kalman Filter Class
class KalmanFilter {
    constructor() {
        // State vector [x, y, theta, v] where v is the velocity
        // Initialize to 0 
        this.x = [0, 0, 0, 0];
        // Covariance matrix
        this.P = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
        // Process noise covariance
        this.Q = [
            [0.01, 0, 0, 0],
            [0, 0.01, 0, 0],
            [0, 0, 0.01, 0],
            [0, 0, 0, 0.01]
        ];
        // Measurement noise covariance
        this.R = [
            [0.1, 0, 0],
            [0, 0.1, 0],
            [0, 0, 0.1]
        ];
        // Measurement matrix
        this.H = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ];
        // Identity matrix
        this.I = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
    }

    // Prediction step
    predict() {
        
    }

    // Update step
    update() {
    }

    // Helper functions for matrix operations
    add(A, B) {
        return A.map((r, i) => r.map((v, j) => v + B[i][j])); 
    }

    subtract(A, B) {
        return A.map((valueA, indexInA) => valueA - B[indexInA]);
    }

    multiply(A, B) {
      var result = [];
      for (var i = 0; i < A.length; i++) {
          result[i] = [];
          for (var j = 0; j < B[0].length; j++) {
              var sum = 0;
              for (var k = 0; k < A[0].length; k++) {
                  sum += A[i][k] * B[k][j];
              }
              result[i][j] = sum;
          }
      }
      return result;
  }

    transpose(M) {
        return M[0].map((_, colIndex) => M.map(row => row[colIndex]));
    }

    // Returns the inverse of matrix `M`.
    inverse(M){
      // I use Guassian Elimination to calculate the inverse:
      //if the matrix isn't square: exit (error)
      if(M.length !== M[0].length){return;}
      
      //create the identity matrix (I), and a copy (C) of the original
      var i=0, ii=0, j=0, dim=M.length, e=0, t=0;
      var I = [], C = [];
      for(i=0; i<dim; i+=1){
          // Create the row
          I[I.length]=[];
          C[C.length]=[];
          for(j=0; j<dim; j+=1){
              
              //if we're on the diagonal, put a 1 (for identity)
              if(i==j){ I[i][j] = 1; }
              else{ I[i][j] = 0; }
              
              // Also, make the copy of the original
              C[i][j] = M[i][j];
          }
      }
      // Perform elementary row operations
      for(i=0; i<dim; i+=1){
          // get the element e on the diagonal
          e = C[i][i];
          // if we have a 0 on the diagonal (we'll need to swap with a lower row)
          if(e==0){
              //look through every row below the i'th row
              for(ii=i+1; ii<dim; ii+=1){
                  //if the ii'th row has a non-0 in the i'th col
                  if(C[ii][i] != 0){
                      //it would make the diagonal have a non-0 so swap it
                      for(j=0; j<dim; j++){
                          e = C[i][j];       //temp store i'th row
                          C[i][j] = C[ii][j];//replace i'th row by ii'th
                          C[ii][j] = e;      //repace ii'th by temp
                          e = I[i][j];       //temp store i'th row
                          I[i][j] = I[ii][j];//replace i'th row by ii'th
                          I[ii][j] = e;      //repace ii'th by temp
                      }
                      //don't bother checking other rows since we've swapped
                      break;
                  }
              }
              //get the new diagonal
              e = C[i][i];
              //if it's still 0, not invertable (error)
              if(e==0){return}
          }
          // Scale this row down by e (so we have a 1 on the diagonal)
          for(j=0; j<dim; j++){
              C[i][j] = C[i][j]/e; //apply to original matrix
              I[i][j] = I[i][j]/e; //apply to identity
          }
          // Subtract this row (scaled appropriately for each row) from ALL of
          // the other rows so that there will be 0's in this column in the
          // rows above and below this one
          for(ii=0; ii<dim; ii++){
              // Only apply to other rows (we want a 1 on the diagonal)
              if(ii==i){continue;}
              
              // We want to change this element to 0
              e = C[ii][i];
              
              // Subtract (the row above(or below) scaled by e) from (the
              // current row) but start at the i'th column and assume all the
              // stuff left of diagonal is 0 (which it should be if we made this
              // algorithm correctly)
              for(j=0; j<dim; j++){
                  C[ii][j] -= e*C[i][j]; //apply to original matrix
                  I[ii][j] -= e*I[i][j]; //apply to identity
              }
          }
      }
      
      //we've done all operations, C should be the identity, matrix I should be the inverse:
      return I;
    }
}



// Global variables
let video, videoSelect, canvas, context, frameData, videoStream;
let sendDataCheck; // stops client sending same data to server multiple times
const interval = 100; // period of sending data back to the server (ms)
const sensorData = new SensorData();
let motorControl = [1, 1, 0]; //left right tosend

// DOM loaded callback
document.addEventListener('DOMContentLoaded', initialiseApp);

function initialiseApp() {
  initialiseElements();
  setupSensors();
  setupMotor();
  setupVideoStream();
}

function initialiseElements() {
  video = document.querySelector('video');
  videoSelect = document.querySelector('select#videoSource');
  canvas = document.getElementById('canvas');
  context = canvas.getContext('2d');
  
  videoSelect.onchange = getVideoStream;
}

// Video processing
function setupVideoStream() {
  getDevices().then(gotDevices).then(getVideoStream);
}

function processVideo() {
  if (video.videoWidth > 0 && video.videoHeight > 0) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    context.drawImage(video, 0, 0, canvas.width, canvas.height);
    frameData = context.getImageData(0, 0, canvas.width, canvas.height);
  }
  requestAnimationFrame(processVideo);
}

// Device handling
async function getDevices() {
  return navigator.mediaDevices.enumerateDevices();
}

function gotDevices(deviceInfos) {
  videoSelect.innerHTML = '';
  for (const deviceInfo of deviceInfos) {
    if (deviceInfo.kind === 'videoinput') {
      const option = document.createElement('option');
      option.value = deviceInfo.deviceId;
      option.text = deviceInfo.label || `Camera ${videoSelect.length + 1}`;
      videoSelect.appendChild(option);
    }
  }
}

async function getVideoStream() {
  if (videoStream) {
    videoStream.getTracks().forEach(track => track.stop());
  }
  const videoSource = videoSelect.value;
  const constraints = {
    video: { deviceId: videoSource ? { exact: videoSource } : undefined }
  };
  try {
    videoStream = await navigator.mediaDevices.getUserMedia(constraints);
    gotVideoStream(videoStream);
  } catch (error) {
    console.error('Error accessing media devices.', error);
  }
}

function gotVideoStream(newVideoStream) {
  videoStream = newVideoStream;
  video.srcObject = videoStream;
  requestAnimationFrame(processVideo);
  const activeTrack = videoStream.getVideoTracks()[0];
  videoSelect.value = activeTrack.getSettings().deviceId;
}

// Sensor handling
function setupSensors() {
  const enableSensorsBtn = document.getElementById('enableSensors');
  if (typeof DeviceMotionEvent.requestPermission === 'function') {
    enableSensorsBtn.style.display = 'block';
    enableSensorsBtn.addEventListener('click', requestIOSPermission);
  } else {
    initialiseSensors();
  }
}

async function requestIOSPermission() {
  try {
    const response = await DeviceMotionEvent.requestPermission();
    if (response === "granted") {
      initialiseSensors();
    }
  } catch (error) {
    console.error("Error requesting iOS permission", error);
  }
}

function initialiseSensors() {
  try {
    window.addEventListener("devicemotion", handleMotion);
    window.addEventListener("deviceorientation", handleOrientation);
  } catch (error) {
    console.error("Sensor APIs not supported", error);
  }
}
//motor button
function setupMotor() {
  const motorBtn = document.getElementById('enableMotors');
  motorBtn.addEventListener('click', motorOffOn);
}
function motorOffOn(){
  motorControl[0] = 1 - motorControl[0];
  motorControl[1] = 1 - motorControl[1];
  motorControl[2] = 1; // send this command
  console.log(motorControl);
}
function handleOrientation(event) {
  // Update the orientation data correctly
  sensorData.updateOrientation(event.alpha, event.beta, event.gamma);
  
  // Also update the display fields
  updateFieldIfNotNull('Orientation_a', event.alpha);
  updateFieldIfNotNull('Orientation_b', event.beta);
  updateFieldIfNotNull('Orientation_g', event.gamma);
}

function handleMotion(event) {
  const { acceleration, rotationRate } = event;
  sensorData.updateAccelerometer(acceleration.x, acceleration.y, acceleration.z);
  sensorData.updateGyroscope(rotationRate.alpha, rotationRate.beta, rotationRate.gamma);
  
  updateFieldIfNotNull('Accelerometer_x', acceleration.x);
  updateFieldIfNotNull('Accelerometer_y', acceleration.y);
  updateFieldIfNotNull('Accelerometer_z', acceleration.z);
  
  updateFieldIfNotNull('Gyroscope_x', rotationRate.alpha);
  updateFieldIfNotNull('Gyroscope_y', rotationRate.beta);
  updateFieldIfNotNull('Gyroscope_z', rotationRate.gamma);
}

function updateFieldIfNotNull(fieldName, value, precision = 3) {
  if (value != null) {
    document.getElementById(fieldName).innerHTML = value.toFixed(precision);
  }
}


// Sending data back to the server
function sendDataToServer(data) {
  fetch('/api/clientData', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data)
  })
  .then(response => response.json())
  .then(result => {
    console.log('Data sent successfully:', result);
  })
  .catch(error => {
    console.error('Error sending data:', error);
  });
}

// New function to start periodic data sending
function startPeriodicDataSending() {
  if (sendDataCheck) {
    clearInterval(sendDataCheck);
  }
  sendDataCheck = setInterval(() => {
    const sensorDataPayload = sensorData.getAllData();
    const motorDataPayload = motorControl;
    const data = {
      sensorData: sensorDataPayload,
      motorControlData: motorDataPayload
    };
    sendDataToServer(data);
    motorControl[2] = 0; //do not send again
  }, interval);
}

// Start sending data immediately after initialization
document.addEventListener('DOMContentLoaded', () => {
  initialiseApp();
  startPeriodicDataSending();
});
