import web_interface_core
import time
import os

client = web_interface_core.DRInterface("JJo1qfmc", "192.168.0.184")
client.log_on()
print("set manual mode " + str(client.set_manual_mode()))
print("start")
client.start_car()
print("stop")
client.stop_car()

model_file_path = "/home/michael/Desktop/LP-filter-2ms.tar.gz"
model_name_with_ext = os.path.basename(model_file_path)
model_name_wout_ext = os.path.splitext(os.path.splitext(model_name_with_ext)[0])[0]
print(model_name_with_ext)
print(model_name_wout_ext)
print("uploading model")
print(client.upload_model(model_file_path, model_name_with_ext))

print("verifying model upload")
model_ready = False
while not model_ready:
    models = client.get_uploaded_models()
    print(models)
    for model in models:
        if model["name"] == model_name_wout_ext and model["status"] == "Ready":
            model_ready = True
    time.sleep(0.5)

print("Model upload verified")

print("logging on again")
client.log_on()
print("logon successful")
print("Set autonomous mode " + str(client.set_autonomous_mode()))

print("loading model")
print(client.load_model(model_name_wout_ext))

client.set_throttle_percent(55)
print("set throttle percent to 55")

print("start")
client.start_car()
time.sleep(10)
print("stop")
client.stop_car()

# #print(client.get_models())
# #print(client.get_is_usb_connected())
# print(client.set_manual_mode())
# client.start_car()
# print(client.send_drive_command(1, 0.05))
# time.sleep(5)
# print(client.stop_car())
