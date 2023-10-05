import yaml


def load_yaml(yaml_file):
    with open(yaml_file, "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

    return config


def calculate_match_rate(result, gt_yaml, field):
    if field == 'environment':
        mismatch = (len(result['environment']['wrong']) + len(result['environment']['missing']) + len(
            result['environment']['redundant']))

        if gt_yaml['environment'] != None:
            total = len(gt_yaml['environment'])
        else:
            total = 0

    elif field == 'road_network':
        mismatch = (len(result['road_network']['wrong']) + len(result['road_network']['missing']) + len(
            result['road_network']['redundant']))

        if gt_yaml['road_network'] != None:
            total = len(gt_yaml['road_network'])
        else:
            total = 0

    elif field == 'participant':
        mismatch = 0
        total = 0
        for sub_field in ['ego_vehicle', 'other_actor']:
            for k, v in gt_yaml['participant'][sub_field].items():
                sub_result = result['participant'][sub_field][k]
                if type(v) == list:
                    total += len(v)
                elif v != None:
                    total += 1

                mismatch += len(sub_result['wrong']) + len(sub_result['missing']) + len(sub_result['redundant'])

    elif field == 'all':
        mismatch_environment, total_environment, _ = calculate_match_rate(result, gt_yaml, 'environment')
        mismatch_road_network, total_road_network, _ = calculate_match_rate(result, gt_yaml, 'road_network')
        mismatch_participant, total_participant, _ = calculate_match_rate(result, gt_yaml, 'participant')
        mismatch = mismatch_environment + mismatch_road_network + mismatch_participant
        total = total_environment + total_road_network + total_participant

    if total != 0:
        match_rate = 1 - mismatch / total
    elif mismatch == 0:
        match_rate = 1
    else:
        match_rate = 0

    return mismatch, total, match_rate


def check_difference(gt_yaml, generated_yaml, field='all'):
    result = {}
    if field == 'environment':
        result['environment'] = check_list_field(gt_yaml['environment'], generated_yaml['environment'])

    elif field == 'road_network':
        result['road_network'] = check_list_field(gt_yaml['road_network'], generated_yaml['road_network'])

    elif field == 'participant':
        result['participant'] = check_participant(gt_yaml['participant'], generated_yaml['participant'])
    elif field == 'all':
        result['environment'] = check_list_field(gt_yaml['environment'], generated_yaml['environment'])
        result['road_network'] = check_list_field(gt_yaml['road_network'], generated_yaml['road_network'])
        result['participant'] = check_participant(gt_yaml['participant'], generated_yaml['participant'])

    _, _, match_rate = calculate_match_rate(result, gt_yaml, field=field)
    print(result)
    print(match_rate)

    return result, match_rate


def check_value_field(gt_info, generated_info):
    result = {'wrong': [], 'missing': [], 'redundant': []}
    if gt_info == None and generated_info != None:
        result['redundant'].append(generated_info)
    elif gt_info != None and generated_info == None:
        result['missing'].append(gt_info)
    elif gt_info != generated_info:
        result['wrong'].append('{}->{}'.format(gt_info, generated_info))

    return result


def check_list_field(gt_info, generated_info):
    if gt_info == None:
        gt_info = []
    if generated_info == None:
        generated_info = []
    inconsistent_gt = [item for item in gt_info if item not in generated_info]
    inconsistent_generated = [item for item in generated_info if item not in gt_info]
    result = {'wrong': [], 'missing': [], 'redundant': []}
    if len(inconsistent_gt) == len(inconsistent_generated):
        # wrong
        for i in range(len(inconsistent_gt)):
            result['wrong'].append('{}->{}'.format(inconsistent_gt[i], inconsistent_generated[i]))
    elif len(inconsistent_gt) == 0:
        # redundant
        for i in range(len(inconsistent_generated)):
            result['redundant'].append(inconsistent_generated[i])
    elif len(inconsistent_generated) == 0:
        # missed
        for i in range(len(inconsistent_gt)):
            result['missing'].append(inconsistent_gt[i])
    else:
        # wrong, and missing or redundant
        for i in range(min(len(inconsistent_generated), len(inconsistent_gt))):
            result['wrong'].append(['{}->{}'.format(inconsistent_gt[i], inconsistent_generated[i])])

        if len(inconsistent_generated) > len(inconsistent_gt):
            for i in range(len(inconsistent_gt), len(inconsistent_generated)):
                result['redundant'].append(inconsistent_generated[i])
        else:
            for i in range(len(inconsistent_generated), len(inconsistent_gt)):
                result['missing'].append(inconsistent_gt[i])

    return result


def check_participant(gt_info, generated_info):
    result = {'ego_vehicle': {}, 'other_actor': {}}
    # ego_vehicle
    for field in ['ego_vehicle', 'other_actor']:
        for k, v in gt_info[field].items():
            if type(v) == list:
                result[field][k] = check_list_field(gt_info[field][k], generated_info[field][k])
            else:
                result[field][k] = check_value_field(gt_info[field][k], generated_info[field][k])
    return result


if __name__ == "__main__":
    num_rules = 21
    total_match_rate = 0
    for i in range(1, num_rules + 1):
        print('rule {}'.format(i))
        ##generated_yaml = load_yaml('generated/rule{}.yaml'.format(i))
        generated_yaml = load_yaml('../my_test/rule{}.yaml'.format(i))
        gt_yaml = load_yaml('../ground_truth/rule{}.yaml'.format(i))
        result, match_rate = check_difference(gt_yaml, generated_yaml, field='environment')
        print('\n')

        total_match_rate = total_match_rate + match_rate
        # break
    print(total_match_rate)
    print('total_match_rate={}'.format(round(total_match_rate / num_rules, 3)))
