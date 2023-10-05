from write import *


def gen_question(question):
    ##return f"Based on the description, \"{rule}\", {question}?, answer with yes or no."
    return f"{question}"


def gen_multi_time_q(item_list):
    """
    rule: the input traffic rule
    item_list: e.g., ['daytime', 'brightness of the sky']
    return: a list of all the questions
    """
    return [
        gen_question(f"is {item_list[0]} mentioned"),
        gen_question(f"is {item_list[0]} explicitly mentioned"),
        gen_question(f"is it {item_list[0]} now"),
        gen_question(f"is there any mention of {item_list[0]}"),
        gen_question(f"is {item_list[1]} explicitly mentioned")
    ]

def gen_multi_pre_weather_q(item_list):
    return [
        gen_question(f"is {item_list[0]} mentioned"),
        gen_question(f"is {item_list[0]} explicitly mentioned"),
        gen_question(f"is {item_list[1]} mentioned"),
        gen_question(f"is {item_list[1]} explicitly mentioned"),
        gen_question(f"is there any mention of {item_list[1]}"),
    ]

def gen_multi_weather_q(item_list):
    return [
        gen_question(f"is it {item_list[0]}"),
        gen_question(f"is it {item_list[1]} now"),
        gen_question(f"is {item_list[2]} mentioned"),
        gen_question(f"is {item_list[2]} explicitly mentioned"),
        gen_question(f"is it in {item_list[2]}"),
    ]


def gen_qs_time_or_weather(keys=['time', 'pre_weather', 'weather']):
    """ time and weather Questions
    keys = ['time', 'pre_weather', 'weather']
    output: [['.. mentioned', '... mentioned', ...5 questions], [5 questions], ...] """

    all_questions_temp=[]

    if 'time' in keys:
        keys_time = [['daytime', 'brightness of the sky'], ['nighttime', 'darkness']]
        for time_item in keys_time:
            all_ans_temp = gen_multi_time_q(time_item)
            all_questions_temp.append(all_ans_temp)

    if 'pre_weather' in keys:
        keys_pre_weather = [['weather', 'weather condition']]
        for pre_weather_item in keys_pre_weather:
            all_ans_temp = gen_multi_pre_weather_q(pre_weather_item)
            all_questions_temp.append(all_ans_temp)

    if 'weather' in keys:
        keys_weather = [['sunny', 'sunny', 'sunny weather'], ['rainy', 'raining', 'rainy weather'],
                        ['snowy', 'snowying', 'snowy weather'], ['foggy', 'foggy', 'foggy weather']]
        for weather_item in keys_weather:
            all_ans_temp = gen_multi_weather_q(weather_item)
            all_questions_temp.append(all_ans_temp)

    return all_questions_temp


def gen_qs_env(keys, gen_q_file):
    #keys = ['time', 'pre_weather', 'weather']
    q_list = gen_qs_time_or_weather(keys)  ## q_list: [[5 qs: daytime], [5 qs: nighttime], [5 qs: pre_weather], ...]

    #gen_q_file = '../text_files/q_env.txt'
    if os.path.exists(gen_q_file):
        os.remove(gen_q_file)

    ## write down the generated questions
    for q_cls in q_list:
        for q in q_cls:
            write_q(gen_q_file, q.strip())


def main_env():
    text_file_path = '../text_files/'
    keys_all = [['time'], ['pre_weather'], ['weather'], ['time', 'pre_weather', 'weather']]
    gen_qs_files = ['q_time.txt', 'q_pre_weather.txt', 'q_weather.txt', 'q_env.txt']
    for i in range(len(keys_all)):
        saving_path = text_file_path + gen_qs_files[i]
        gen_qs_env(keys_all[i], saving_path)
        print('Done: generating questions: {}'.format(saving_path))


if __name__ == "__main__":
    main_env()