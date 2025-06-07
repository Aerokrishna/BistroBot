#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, jsonify
from kpbot_interface.srv import DeliveryPlan  # custom service with location_id[] and tray_number[]
import os
import threading

class DeliveryUI(Node):
    def __init__(self):
        super().__init__('delivery_ui')

        self.cli = self.create_client(DeliveryPlan, 'start_delivery')
        self.req = DeliveryPlan.Request()
        self.delivery_plan = {1: [], 2: [], 3: []}  # Tray to locations

# Get absolute path to templates folder relative to this script file
template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
app = Flask(__name__, template_folder=template_dir)

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/tray_select')
def tray_select():
    # Print tray and corresponding location ids to the console
    for tray, locs in node.delivery_plan.items():
        print(f"Tray {tray}: Location IDs: {locs}")
    return render_template('tray_select.html', delivery_plan=node.delivery_plan)

@app.route('/select_locations/<int:tray_id>')
def select_locations(tray_id):
    selected_locations = node.delivery_plan.get(tray_id, [])
    return render_template('location_select.html', tray_id=tray_id, selected_locations=selected_locations)

@app.route('/confirm_locations/<int:tray_id>', methods=['POST'])
def confirm_locations(tray_id):
    location_ids = request.json.get('locations', [])
    node.delivery_plan[tray_id] = location_ids
    return jsonify({'success': True})

@app.route('/start_delivery', methods=['POST'])
def start_delivery():
    location_ids = []
    tray_numbers = []

    for tray, locs in node.delivery_plan.items():
        for loc in locs:
            location_ids.append(int(loc))
            tray_numbers.append(int(tray))

    node.req.location_ids = location_ids
    node.req.tray_number = tray_numbers

    future = node.cli.call_async(node.req)

    return jsonify({'success': True, 'message': 'Delivery started!'})

def run_flask():
    app.run(host='0.0.0.0', port=3000)

def main(args=None):
    global node
    rclpy.init(args=args)
    node = DeliveryUI()

    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    flask_thread.join()

if __name__ == '__main__':
    main()
