
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

def LoadInstance(file_path):
    """
    Load the instance file and return a dictionary
    """
    inst = dict()

    # Read the file line by line
    with open(file_path, 'r') as file:
        lines = file.readlines()

        # Extract relevant information from each line
        for line in lines[0:1]:
            line = line.strip()  # Remove leading/trailing whitespace
            #print(line)
            parts = line.split()
            #print(parts)
            row=int((parts[0].split(':')[-1]))
            col=int(parts[1].split(':')[-1])
            #print(row)
            #print(col)
            inst['Row']=row
            inst['Column']=col

        grid=[[0]*col for _ in range(row)]
        k=0
        for line in lines[1:row+1]:
            line=line.strip()
            for j in range(col):
                grid[k][j]=int(line[j])

            k=k+1
        inst['Map']=grid
        #print(inst["Map"][1])

        for line in lines[row+1:row+3]:
            #print(type(line))
            line.strip()
            if "Start" in line:
                start_coordinates = line.split(":")[1].strip("()").split(",")
                start_coordinates[1]=start_coordinates[1][:-2]
                #print(start_coordinates)
                x, y = int(start_coordinates[0]),int(start_coordinates[1])
                inst["Start"] = (x, y)
            elif "Goal" in line:
    # 如果字符串包含 "goal"，则提取坐标值
                goal_coordinates = line.split(":")[1].strip("()").split(",")
                goal_coordinates[1]=goal_coordinates[1][:-2]
                x, y = int (goal_coordinates[0]),int (goal_coordinates[1])
                inst["Goal"] = (x, y)
        box=set()
        for line in lines[row+4:]:
            box_pos=line.strip("()").split(",")
            box_pos[1]=box_pos[1][:-2]
            x,y=int(box_pos[0]),int(box_pos[1])
            box.add((x,y))

        inst["Boxes"]=box
    return inst

def PathVis(res, inst,folder):

    # 逐步绘制机器人路径和箱子位置
    for label in res:

        if label.startswith("LABEL"):
            label_data = res[label]
            robot_path = label_data["robot"]
            boxes = label_data["box"]
            label_folder=os.path.join(folder,label)
            if not os.path.exists(label_folder):
                os.makedirs(label_folder)
            # 绘制机器人路径
            i=0
            lx = []
            ly = []

            # extract the path for visualization later
            for robot_coords in robot_path:
                lx.append(robot_coords[0])
                ly.append(robot_coords[1])

            # plot each frame
            for robot_coords in robot_path:
                # start
                plt.scatter(robot_path[0][0], robot_path[0][1], color="red", marker="o", s=100, alpha=0.3)
                # current position
                plt.scatter(robot_coords[0], robot_coords[1], color="red", marker="o", s=100)
                # path
                plt.plot(lx, ly, '--', color="red")
                if len(boxes)>0:
                    box=boxes.pop(0)
                    for bx in box:
                        plt.scatter(bx[0], bx[1], color="green", marker="s", s=100)
                                # 将列表转换为NumPy数组
                    map_arr = np.array(inst["Map"])
                    # map_arr = np.transpose(map_arr)
                    inverted_map = np.where(map_arr == 0, 255, 0)

                    # 创建一个新的图像
                    plt.imshow(inverted_map, cmap="gray", interpolation="nearest")
                    # plt.axis("off")

                # 绘制Goal, red star
                goal_coords = inst["Goal"]
                plt.scatter(goal_coords[0], goal_coords[1], color="red", marker="*", s=100)
                image_path=os.path.join(label_folder,f"sim_"+format(i, '05d')+".png")
                i=i+1
                plt.savefig(image_path)
                plt.close()

def main():
    instance_file="../data/sample_instance.txt"
    inst=LoadInstance(instance_file)
    result_file="../data/sample_result.txt"
    res=LoadResult(result_file)
    folder="../data/sim/"
    PathVis(res,inst,folder)
    


if __name__ == "__main__":

    main()


