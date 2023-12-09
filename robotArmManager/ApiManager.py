from flask import Flask, request
import threading


app = Flask(__name__)
robot_controller = None
    

@app.route('/move', methods=['POST'])
def move_robot_arm():
    data = request.get_json()
    angles = data.get('angles')
    if angles is not None:
        # write in file        
        with open("log.txt") as f:
            lines = f.readlines()
            lines[0] = angles
        with open("log.txt", "w") as f:
            f.writelines(lines)
    
    return 'Success', 200


def flask_thread():
    app.run(host='0.0.0.0', port=5002)


def main():
    thread = threading.Thread(target=flask_thread)
    thread.start()


if __name__ == '__main__':
    main()