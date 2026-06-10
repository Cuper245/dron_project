# server.py
from flask import Flask, jsonify, request
from flask_cors import CORS
from mission import MissionController

app = Flask(__name__)
CORS(app)
mission = MissionController()

@app.route("/stations", methods=["GET"])
def get_stations():
    return jsonify({
        "stations": [
            {"id": k, "position": v, "available": True}
            for k, v in mission.station_map.items()
        ]
    })

@app.route("/dispatch", methods=["POST"])
def dispatch():
    data = request.json
    result = mission.dispatch(data["station_id"])
    return jsonify(result)

@app.route("/status", methods=["GET"])
def status():
    return jsonify({"state": mission.state.value, "target": mission.target_id})

@app.route("/position", methods=["GET"])
def position():
    return jsonify({
        "pos": mission.drone_pos,
        "yaw": mission.drone_yaw,
        "vicon_ok": mission.vicon.get("vicon_ok", False)
    })

# server.py  — add these lines at the very bottom
if __name__ == "__main__":
    app.run(port=5000, debug=False)