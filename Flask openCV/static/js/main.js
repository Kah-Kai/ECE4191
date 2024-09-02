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
const sensorData = new SensorData();

/////////////////////////////////////////////////////
///////////// GLOBALS//////////////////////////////// 
/////////////////////////////////////////////////////
const interval = 1000; // period of sending data back to the server (ms)

/////////////////////////////////////////////////////
///////////// STARTING FUNCTION ///////////////////// 
/////////////////////////////////////////////////////

function openCvReady() {
    cv['onRuntimeInitialized'] = () => {
        setupCameraSelection();
        initialiseSensors();
        startPeriodicDataSending();
    };
}

/////////////////////////////////////////////////////
///////////// Camera Handling  ////////////////////// 
/////////////////////////////////////////////////////

function setupCameraSelection() {
    const videoSourceSelect = document.getElementById("videoSource");

    // Get the list of video input devices
    navigator.mediaDevices.enumerateDevices()
    .then(function(devices) {
        devices.forEach(function(device) {
            if (device.kind === 'videoinput') {
                const option = document.createElement('option');
                option.value = device.deviceId;
                option.text = device.label || `Camera ${videoSourceSelect.length + 1}`;
                videoSourceSelect.appendChild(option);
            }
        });
    });

    videoSourceSelect.onchange = () => {
        start_camera(videoSourceSelect.value);
    };

    // Start the camera with the default device
    start_camera();
}

let videoStream;  // Store the current video stream
let src, dst, hsv, mask, cap;

function start_camera(deviceId = null) {
    let video = document.getElementById("cam_input");
    video.style.display="none"; // removes raw video stream from HTML

    // Stop any existing stream
    if (videoStream) {
        videoStream.getTracks().forEach(track => track.stop());
    }

    // Get the video stream with the selected camera
    const constraints = {
        video: deviceId ? { deviceId: { exact: deviceId } } : true,
        audio: false
    };

    navigator.mediaDevices.getUserMedia(constraints)
    .then(function(stream) {
        videoStream = stream;
        video.srcObject = stream;
        video.play();

        // Initialize OpenCV Mats when the video metadata is loaded
        video.onloadedmetadata = function() {
            if (!src) {  // Avoid re-initializing Mats
                src = new cv.Mat(video.height, video.width, cv.CV_8UC4);
                dst = new cv.Mat(video.height, video.width, cv.CV_8UC4);
                hsv = new cv.Mat(video.height, video.width, cv.CV_8UC3);
                mask = new cv.Mat(video.height, video.width, cv.CV_8UC1);
                cap = new cv.VideoCapture(video);
                processVideo();  // Start processing video
            }
        };
    })
    .catch(function(err) {
        console.log("An error occurred! " + err);
    });
}

function processVideo() {
    const FPS = 60;

    function processFrame() { // local function definition lmao
        let begin = Date.now();
        cap.read(src);

        // Convert image to HSV color space
        cv.cvtColor(src, hsv, cv.COLOR_RGBA2RGB);
        cv.cvtColor(hsv, hsv, cv.COLOR_RGB2HSV);

        // Define range of red color in HSV
        let low = new cv.Mat(hsv.rows, hsv.cols, hsv.type(), [0, 100, 100, 0]);
        let high = new cv.Mat(hsv.rows, hsv.cols, hsv.type(), [10, 255, 255, 255]);

        // Threshold the HSV image to get only red colors
        cv.inRange(hsv, low, high, mask);

        // Find contours
        let contours = new cv.MatVector();
        let hierarchy = new cv.Mat();
        cv.findContours(mask, contours, hierarchy, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (assuming it's the ball)
        let maxArea = 0;
        let maxContourIndex = -1;
        for (let i = 0; i < contours.size(); ++i) {
            let area = cv.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = area;
                maxContourIndex = i;
            }
        }

        // Draw circle around the ball if found
        if (maxContourIndex !== -1) {
            let ballContour = contours.get(maxContourIndex);
            let circle = cv.minEnclosingCircle(ballContour);
            let center = new cv.Point(Math.round(circle.center.x), Math.round(circle.center.y));
            let radius = Math.round(circle.radius);
            cv.circle(src, center, radius, [0, 255, 0, 255], 2);
        }

        cv.imshow("canvas_output", src);

        // Clean up
        low.delete(); high.delete();
        contours.delete(); hierarchy.delete();

        // Schedule next frame
        let delay = 1000 / FPS - (Date.now() - begin);
        setTimeout(processFrame, delay);
    }

    // Start processing frames
    processFrame();
}


/////////////////////////////////////////////////////
///////////// Sensor Handling  ////////////////////// 
/////////////////////////////////////////////////////

function initialiseSensors() {
    try {
      window.addEventListener("devicemotion", handleSensorUpdate);
      window.addEventListener("deviceorientation", handleSensorUpdate);
    } catch (error) {
      console.error("Sensor APIs not supported", error);
    }
  }
  
  function handleSensorUpdate(event){
    if (event.type === 'deviceorientation') {
      // Update the orientation data correctly
      sensorData.updateOrientation(event.alpha, event.beta, event.gamma);
        
      // Also update the display fields
      updateFieldIfNotNull('Orientation_a', event.alpha);
      updateFieldIfNotNull('Orientation_b', event.beta);
      updateFieldIfNotNull('Orientation_g', event.gamma);
    } else if(event.type === 'devicemotion'){
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
  }
  
  function updateFieldIfNotNull(fieldName, value, precision = 3) {
    if (value != null) {
      document.getElementById(fieldName).innerHTML = value.toFixed(precision);
    }
  }

/////////////////////////////////////////////////////
///////////// HOST-CLIENT COMMS ////////////////////// 
/////////////////////////////////////////////////////
let sendDataCheck; // stops client sending same data to server multiple times

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
      const data = {
        sensorData: sensorDataPayload
      };
      sendDataToServer(data);
    }, interval);
  }