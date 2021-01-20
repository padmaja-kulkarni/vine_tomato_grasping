import os
import json

def main():
    i_start = 1
    i_end = 85
    n = i_end - i_start

    key = 'imageData'
    path_root = os.path.join(os.sep, "media", "taeke", "backup", "thesis_data", "detect_truss")
    path = os.path.join(path_root, "data", "lidl")

    file_names = []
    for count, i_truss in enumerate(range(i_start, i_end)):
        truss_name = str(i_truss).zfill(3)
        file_name = os.path.join(path, truss_name + '.json')
        file_names.append(file_name)

    user_input = raw_input("{1}Are you sure to delete all {0} data from the above mentioned json files [y/n]? This "
                           "cannot be undone:\n".format(key, "\n".join(file_names) + "\n"))
    if user_input == 'y':
        for file_name in file_names:
            print("About to remove {0} key from file {1}".format(key, file_name))
            remove_key_from_json(key, file_name)
    elif user_input == 'n':
        print "Will not execute"
    else:
        print("Unknown input, will not execute")


def remove_key_from_json(key, path_file):

    if not os.path.isfile(path_file):
        print("json does not exist on path {0} skipping this file!".format(path_file))
        return None

    with open(path_file, "r") as read_file:
        json_data = json.load(read_file)

    if key not in json_data.keys():
        print("Specified key {0} is not present in json file, skipping this file!".format(key))
        return None

    result = json_data.pop(key, None)
    if result is None:
        print("Failed to pop {0} from dictionary".format(key))
        return None
    else:
        print("Successfully popped {0} from dictionary".format(key))

    with open(path_file, "w") as write_file:
        json.dump(json_data, write_file, indent=4)

    return result


if __name__ == '__main__':
    main()