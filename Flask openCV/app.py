from flask import Flask, render_template, request, jsonify
###############
## FLASK APP ##
###############

# Initialise Flask app
app = Flask(__name__)
@app.route('/')
def home():
    return render_template('index.html')
    
@app.route('/api/clientData', methods=['POST'])

def receive_data():
    data = request.json
    sensorDataRecieved = data.get('sensorData')
    print(sensorDataRecieved)
    return jsonify({"message": "Sensor Data received successfully"})

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=6969, debug=False)  # Use debug=False for production
    except KeyboardInterrupt:
        pass
    finally:
        pass