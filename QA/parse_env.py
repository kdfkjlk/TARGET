from run_GPT3 import *
from write import *


def ask_questions(rule, question_file_name):
    ans_list =[]; prob_ans_list =[]
    with open(question_file_name, 'r') as f:
        for line in f.readlines():
            question = line.strip('\n')
            ans, prob = run_gpt3(rule, question)
            ans_list.append(ans)
            prob_ans_list.append(prob)
            time.sleep(2)
    return ans_list, prob_ans_list


def divide_ans(ans_list, prob_ans_list, divide_lens):
    ''' devide answers based on questions, e.g., sun, rain, snow, ...
    divide_lens is a list of the length of the same group of questions
    output: ans_groups, prob_groups: e.g., [['yes', 'yes', 'no'], ['no', 'yes', 'no']]'''
    ans_groups = []; prob_groups = []
    start_idx = 0; end_idx = 0

    for i in divide_lens:
        end_idx += i
        ans_groups.append(ans_list[start_idx : end_idx])
        prob_groups.append(prob_ans_list[start_idx : end_idx])
        start_idx += end_idx

    return ans_groups, prob_groups


def vote_majority(ans_groups, prob_groups):
    ## for each answer, calculate the ratio of 'yes' and the avg probability
    ## output: e.g., correspond to [ans: day, night], ans_ratio=[0.8, 0.3], ans_avg_prob_list=[0.85, 0.79]
    ans_ratio_list = []; ans_avg_prob_list =[]

    for i in range(len(ans_groups)):
        # calculate ratio
        ans_ratio = ans_groups[i].count('yes') /(len(ans_groups[i] ) +0.00001)
        ans_ratio_list.append(ans_ratio)

        # calculate avg prob
        ans_prob_temp = []
        for j in range(len(ans_groups[i])):
            if ans_groups[i][j] == 'yes':
                ans_prob_temp.append(prob_groups[i][j])

        avg_prob_temp = sum(ans_prob_temp ) /(len(ans_prob_temp ) +0.00001)
        ans_avg_prob_list.append(avg_prob_temp)

    return ans_ratio_list, ans_avg_prob_list


def compare_ans(ans_ratio_list, ans_avg_prob_list, prob_threshold):
    ## compare based on avg_prob
    ans_yes_prob = []

    # create the prob of answer 'yes' for each kind of questions, e.g., day, night
    for i in range(len(ans_ratio_list)):
        if (ans_ratio_list[i] > 0.5) and (ans_avg_prob_list[i] > prob_threshold):
            ans_yes_prob.append(float(ans_avg_prob_list[i]))
        else:
            ans_yes_prob.append(-100)

    ## obtain the maxmum probability
    max_prob_idx = ans_yes_prob.index(max(ans_yes_prob))
    max_prob = max(ans_yes_prob)
    ratio_ans = ans_ratio_list[max_prob_idx]

    return max_prob_idx, ratio_ans, max_prob


def implement_env(rule, env, ans_option, question_file_name, ans_group_num, env_option, prob_threshold=0.75):
    #print('parse_env_original.py, line75, function implement_env, env_option={}'.format(env_option))
    ans_list, prob_ans_list = ask_questions(rule, question_file_name)
    #print('parse_env_original.py, line76, implement_env, ans_list={}, prob_list={}\n'.format(ans_list, prob_ans_list))
    ans_groups, prob_groups = divide_ans(ans_list, prob_ans_list, ans_group_num)
    ans_ratio_list, ans_avg_prob_list = vote_majority(ans_groups, prob_groups)
    max_prob_idx, ratio_ans, max_prob = compare_ans(ans_ratio_list, ans_avg_prob_list, prob_threshold)

    if max_prob > prob_threshold:
        env[env_option] = ans_option[max_prob_idx]

    return ans_list, prob_ans_list



def parse_env_one_rule(rule, env, time_qs_file_name, pre_weather_qs_file_name, weather_qs_file_name):
    prob_threshold = 0.75

    ## to excel
    time_table = []
    pre_weather_table = []
    weather_table = []

    ## time
    time_option = ['Day', 'night']
    ans_list, prob_ans_list = implement_env(rule, env, time_option, time_qs_file_name, [5]*len(time_option), 'time', prob_threshold)

    for i in range(len(ans_list)):
        time_table.append(ans_list[i])
        time_table.append(prob_ans_list[i])

    ## pre_weather
    ans_list, prob_ans_list = ask_questions(rule, pre_weather_qs_file_name)
    ans_groups, prob_groups = divide_ans(ans_list, prob_ans_list, [1])
    ans_ratio_list, ans_avg_prob_list = vote_majority(ans_groups, prob_groups)

    for i in range(len(ans_list)):
        pre_weather_table.append(ans_list[i])
        pre_weather_table.append(prob_ans_list[i])

    ## weather
    if (ans_ratio_list[0] > 0.5) and (ans_avg_prob_list[0] > prob_threshold):
        weather_option = ['Sunny', 'wet', 'Snowy', 'foggy']
        ans_w_list, prob_ans_w_list = implement_env(rule, env, weather_option, weather_qs_file_name, [5]*len(weather_option), 'weather', prob_threshold)

        for i in range(len(ans_w_list)):
            weather_table.append(ans_w_list[i])
            weather_table.append(prob_ans_w_list[i])

    return time_table, pre_weather_table, weather_table


def generate_yaml_data(input_rule, time_qs_file_name, pre_weather_qs_file_name, weather_qs_file_name):
    env = {'time': None, 'weather': None}
    print(input_rule)

    input_list_items_time, input_list_items_pre_weather, input_list_items_weather = parse_env_one_rule(input_rule, env, time_qs_file_name, pre_weather_qs_file_name, weather_qs_file_name)

    # road = parse_road_network()
    # ego, other = parse_participant()
    road = {}
    ego = {'behavior': None}
    other = {'behavior': None}

    #print(road, env, ego, other)
    return [road, env, ego, other], input_list_items_time, input_list_items_pre_weather, input_list_items_weather


def parse_env(text_path):
    idx = 1
    ##result_folder = 'generated'
    result_folder = '../my_test'
    rule_file_name = text_path + 'rules.txt'
    time_qs_file_name = text_path + 'q_time.txt'
    pre_weather_qs_file_name = text_path + 'q_pre_weather.txt'
    weather_qs_file_name = text_path + 'q_weather.txt'
    table_time_file_name = text_path + 'time_ans_prob'
    table_pre_weather_file_name = text_path + 'pre_weather_ans_prob'
    table_weather_file_name = text_path + 'weather_ans_prob'

    input_list_time = []
    input_list_pre_weather = []
    input_list_weather = []

    with open(rule_file_name, 'r') as f:
        for line in f.readlines():
            rule = line.strip('\n')
            print('rule: {}'.format(idx))

            data_to_create_yaml, input_list_item_time, input_list_items_pre_weather, input_list_item_weather = generate_yaml_data(rule, time_qs_file_name, pre_weather_qs_file_name, weather_qs_file_name)
            create_write_yaml(result_folder, idx, data_to_create_yaml)

            idx += 1
            input_list_time.append(input_list_item_time)
            input_list_pre_weather.append(input_list_items_pre_weather)
            input_list_weather.append(input_list_item_weather)
            print('\n')

    #print(input_list)
    # write_table(input_list_time, table_time_file_name)
    # write_table(input_list_pre_weather, table_pre_weather_file_name)
    # write_table(input_list_weather, table_weather_file_name)

    return data_to_create_yaml


if __name__ == "__main__":
    path = '../text_files/'
    parse_env(path)