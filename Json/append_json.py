import json

def remove_last_brace(json_file):
    with open(json_file, 'r+') as f:
        content = f.read()
        f.seek(0)
        f.write(content[:-1])
        f.truncate()

def remove_first_brace(json_file):
    with open(json_file, 'r+') as f:
        content = f.read()
        f.seek(0)
        f.write(content[1:])
        f.truncate()

def append_files(json_file1, json_file2):
    with open(json_file1, 'a') as dest:
        with open(json_file2, 'r') as src:
            content = src.read()  # Remove braces from the second file content
            dest.write(content)

# Usage
json_file1 = r'C:\Users\Administrator\Desktop\Master\Code\Software-Code\Json\shuttle_0.json'
json_file2 = r'C:\Users\Administrator\Desktop\Master\Code\Software-Code\Json\Model_Generator.json'

remove_last_brace(json_file1)
remove_first_brace(json_file2)
append_files(json_file1, json_file2)
