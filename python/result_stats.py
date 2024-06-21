
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
# add more import here if needed

def LoadResult(file_path):
    """
    Load the result file and return a dictionary with label details.
    """

    res = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()

        # Process the initial key-value pairs
        for line in lines[1:10]:
            key, value = line.strip().split(':')
            res[key.strip()] = float(value.strip())

        # Process the lines starting from the 11th line
        i = 10  # Start from line 11
        while i < len(lines):
            # Extract label number
            label_line = lines[i].strip()
            label_num = label_line.split(':')[1].strip()
            current_label = f"LABEL{label_num}"
            i += 1

            # Extract cost
            cost_line = lines[i].strip().strip('[]').split(',')
            cost = [float(x) for x in cost_line if x]
            i += 1

            # Initialize the label details
            label_details = {
                'label': int(label_num),
                'cost': cost,
                'robot': [],
                
            }

            # Process the robot and boxes line
            robot_boxes_line = lines[i].strip('{')
            robot_boxes_line=robot_boxes_line[:-4]
            #print(robot_boxes_line)
            robot_boxes_data = robot_boxes_line.split('},{')
            #print(robot_boxes_data)
            BOX=[]
            for item in robot_boxes_data:
                if 'robot' in item:
                    robot_data = item.split('[')[1].split(']')[0]
                    label_details['robot'].append([int(x) for x in robot_data.split(',') if x])
                if 'boxes' in item:
                    
                    # Extract boxes data after 'boxes:' and split by '],['
                    boxes_data = item.split('boxes:')[1]
                    boxes_data=boxes_data[:-2]
                    #print(boxes_data)
                    boxes_list = boxes_data.split('],[')
                    #print(boxes_list)
                    nested_boxes = []
                    for box_group in boxes_list:
                        box_coords = box_group.strip('[]')
                        #print(f"box_coords: {box_coords}")
                        if box_coords:  # Ensure the string is not empty
                            nested_boxes.append([int(x) for x in box_coords.split(',') if x])
                    BOX.append(nested_boxes)
            label_details["box"]=BOX

            # Add the current label's details to the main dictionary
            res[current_label] = label_details
            i += 1  # Move to the next label block

    return res

def LoadResults(result_folder, n_boxes, n_instances, map_str, planner_type, push_limit):
    
    out = dict()

    out["runtime_average"] = 0
    out["runtime_median"] = 0
    out["runtime_max"] = 0
    out["runtime_min"] = 0

    out["num_sol_average"] = 0
    out["num_sol_median"] = 0
    out["num_sol_max"] = 0
    out["num_sol_min"] = 0

    out["num_labelExp_average"] = 0
    out["num_labelExp_median"] = 0
    out["num_labelExp_max"] = 0
    out["num_labelExp_min"] = 0

    out["num_labelGen_average"] = 0
    out["num_labelGen_median"] = 0
    out["num_labelGen_max"] = 0
    out["num_labelGen_min"] = 0

    for idx in range(n_instances):
        result_name = result_folder + map_str + "_nbox_" + str(n_boxes) + "_inst_" + \
            str(idx) + "_pushLim_" + str(push_limit) + "_planner_" + str(planner_type) + "_result.txt"
        res = LoadResult(result_name)

        # TODO
        # get the correspoding statistics as specified in the "out" dict from the result 
        # e.g. out["runtime_average"] = (calculate the average runtime of all the instances)

    return out

def main():
    result_folder="../data/result_20240526/"
    n_boxes = 19
    n_instances = 10
    map_str = "empty-8-8"
    planner_type = 0
    push_limit = -1
    out = LoadResults(result_folder, n_boxes, n_instances, map_str, planner_type, push_limit)
    print(out)


if __name__ == "__main__":

    main()


