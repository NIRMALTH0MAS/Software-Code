import json

dic = []
with open('model.json') as file : 
    print("Model Read")
    for i in file:
        dic.append(json.loads(i))
        print(dic)
        # print(json.load(i))
file.close()