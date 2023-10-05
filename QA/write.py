import yaml
import os
from openpyxl import Workbook


## record answer and probability in table
def write_table(input_list, output_file_name):                                  
    book = Workbook()
    sheet = book.active
    # rows = [(88, 46, 57), (89, 38, 12), (23, 59, 78)]
    for row in input_list:
        sheet.append(row)
    book.save('{}.xlsx'.format(output_file_name))


## write down questions
def write_q(q_text_name, q):
    f = open(q_text_name, 'a')
    f.write('{}\n'.format(q))
    f.close()


## write yaml_data into yaml file
def create_write_yaml(yaml_abs_path, idx, data):
    yaml_path = os.path.join(yaml_abs_path, "rule{}.yaml".format(idx))
    road, env, ego, other = data[0], data[1], data[2], data[3]

    ## convert 'behavior' of ego and other_actor from string into list
    if ego['behavior'] is not None:
        ego['behavior'] = ego['behavior'].split('\n ')
    if other['behavior'] is not None:
        other['behavior'] = other['behavior'].split('\n ')

    # generate data for yaml
    if len(road) == 0:
        road = None
    # if len(env)==0:
    #     env=None
    # if len(other)==0:
    #     other=None
    env_total = []
    if env['time']:
        env_total.append(env['time'])
    if env['weather']:
        env_total.append(env['weather'])
    if len(env_total) == 0:
        env_total = None
    data_yaml = {'road_network': road, 'environment': env_total,
                 'participant': {'ego_vehicle': ego, 'other_actor': other}}

    # write data_yaml into yaml file
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(data_yaml, f)