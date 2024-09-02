// classes.js
export class SensorData {
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