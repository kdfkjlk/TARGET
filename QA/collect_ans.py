from compare import load_yaml
from run_GPT3 import run_gpt3
from gen_env_qs import gen_qs_time_or_weather
from gen_road_qs import parse_road_network
from gen_actor_qs import gen_other_actor_exist_qs, gen_other_actor_behavior_qs, gen_other_actor_position_qs
import time
import pickle


def collect_actor_ans():
    rules_file = open('../rules.txt', 'r')
    rules = rules_file.readlines()
    result_file = open('../gpt3_ans/actor.txt', 'w')
    result_total = {}

    for i, rule in enumerate(rules):
        if i >= 4:
            rule_idx = i + 4
        else:
            rule_idx = i + 1

        gt_yaml = load_yaml('ground_truth/rule{}.yaml'.format(rule_idx+1))
        road_networks = gt_yaml['road_network']
        other_actor_info = gt_yaml['participant']['other_actor']
        rule_result = {}
        qs_list = []
        actor_exist_qs = gen_other_actor_exist_qs()
        qs_list += actor_exist_qs

        if other_actor_info['type'] is not None:
            actor_behavior_qs = gen_other_actor_behavior_qs(other_actor_info['type'])
            actor_position_qs = gen_other_actor_position_qs(other_actor_info['type'], road_networks)

            qs_list += actor_behavior_qs
            qs_list += actor_position_qs

        actor_questions = []
        for wq in qs_list:
            actor_questions += wq

        for question in actor_questions:
            ans, confidence = run_gpt3(rule[:-1], question)
            rule_result[question] = [ans, confidence]
            time.sleep(3)
        result_file.write(str(rule_result))
        result_file.flush()
        result_total[rule] = rule_result

    with open('../gpt3_ans/actor.pickle', 'wb') as handle:
        pickle.dump(result_total, handle, protocol=pickle.HIGHEST_PROTOCOL)
        

def collect_road_ans():
    road_questions_group = parse_road_network()
    # print(road_questions_group)
    road_questions = []
    for wq in road_questions_group:
        road_questions += wq
    print(road_questions)
    # print(weather_questions)
    rules_file = open('../rules.txt', 'r')
    rules = rules_file.readlines()
    print(rules)
    result_file = open('../gpt3_ans/road.txt', 'w')
    result_total = {}
    # for each rule, ask all questions and save answers
    for i, rule in enumerate(rules):
        result_file.write(rule)
        rule_result = {}
        for question in road_questions:
            ans, confidence = run_gpt3(rule[:-1], question)
            rule_result[question] = [ans, confidence]
            time.sleep(3)
        result_file.write(str(rule_result))
        result_file.flush()
        result_total[rule] = rule_result

    with open('../gpt3_ans/road.pickle', 'wb') as handle:
        pickle.dump(result_total, handle, protocol=pickle.HIGHEST_PROTOCOL)


def collect_weather_ans():
    # f = open('../gpt3_ans/weather.txt', 'w')
    weather_questions_group = gen_qs_time_or_weather()
    weather_questions = []
    for wq in weather_questions_group:
        weather_questions += wq
    print(weather_questions)
    # print(weather_questions)
    rules_file = open('../rules.txt', 'r')
    rules = rules_file.readlines()
    print(rules)
    result_file = open('../gpt3_ans/weather.txt', 'w')
    result_total = {}
    # for each rule, ask all questions and save answers
    for i, rule in enumerate(rules):
        result_file.write(rule)
        rule_result = {}
        for question in weather_questions:
            ans, confidence = run_gpt3(rule[:-1], question)
            rule_result[question] = [ans, confidence]
            time.sleep(3)
        result_file.write(str(rule_result))
        result_file.flush()
        result_total[rule] = rule_result

    with open('../gpt3_ans/weather.pickle', 'wb') as handle:
        pickle.dump(result_total, handle, protocol=pickle.HIGHEST_PROTOCOL)
    #     pass


# collect_road_ans()
collect_weather_ans()