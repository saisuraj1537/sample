# import required modules
import firebase_admin
from firebase_admin import db,credentials
# authenticate to firebase
cred = credentials.Certificate("/home/dronepi/cred.json")
firebase_admin.initialize_app(cred, {"databaseURL": "https://sample-a028e-default-rtdb.firebaseio.com/"})

ref = db.reference("/")
# retrieving data from root node
ref.get()

print(ref.get())

data = db.reference("/").get()
i=0
while i == 0:
    for key, value in data.items():
        if (data[key]['dronestatus']) == "Not Delivered":
            username = (data[key]['userDetails']['username'])
            shop_lat = (data[key]['shopDetails']['shopLat'])
            shop_long = (data[key]['shopDetails']['shopLong'])
            user_lat = (data[key]['userDetails']['userLat'])
            user_long = (data[key]['userDetails']['userLong'])
            # db.reference("/" + username + "/" + 'dronestatus').set("none")
            i = 1
            break
        
print(shop_lat)
print(shop_long)

# set operation
# db.reference("/" + username + "/" + 'dronestatus').set("none")
# print(ref.get())
# print(long)
# 'All About Python'
# # set operation
# db.reference("/videos").set(3)
# ref.get()
# {'language': 'Python',
#  'name': 'All About Python',
#  'title_count': 0,
#  'titles': ['send email in python', 'create excel in python'],
#  'url': 'https://youtube.com/AllAboutPython',
#  'videos': 3}
# # update operation (update existing value)
# db.reference("/").update({"language": "python"})
# ref.get()
# {'language': 'python',
#  'name': 'All About Python',
#  'title_count': 0,
#  'titles': ['send email in python', 'create excel in python'],
#  'url': 'https://youtube.com/AllAboutPython',
#  'videos': 3}
# # update operation (add new key value)
# db.reference("/").update({"subscribed": True})
# ref.get()
# {'language': 'python',
#  'name': 'All About Python',
#  'subscribed': True,
#  'title_count': 0,
#  'titles': ['send email in python', 'create excel in python'],
#  'url': 'https://youtube.com/AllAboutPython',
#  'videos': 3}
# # push operation
# db.reference("/titles").push().set("create modern ui in python")
# ref.get()
# {'language': 'python',
#  'name': 'All About Python',
#  'subscribed': True,
#  'title_count': 0,
#  'titles': {'0': 'send email in python',
#   '1': 'create excel in python',
#   '-NS_AHc4EiFqorc47wPo': 'create modern ui in python'},
#  'url': 'https://youtube.com/AllAboutPython',
#  'videos': 3}
# # transaction
# def increment_transaction(current_val):
#     return current_val + 1

# db.reference("/title_count").transaction(increment_transaction)
# ref.get()
# {'language': 'python',
#  'name': 'All About Python',
#  'subscribed': True,
#  'title_count': 1,
#  'titles': {'0': 'send email in python',
#   '1': 'create excel in python',
#   '-NS_AHc4EiFqorc47wPo': 'create modern ui in python'},
#  'url': 'https://youtube.com/AllAboutPython',
#  'videos': 3}
# print(ref.key)