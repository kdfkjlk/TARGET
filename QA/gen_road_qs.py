from write import *

def parse_road_network():
    """ Road Network Questions """
    # keys = ['intersection', 'traffic_signals', 'lane', 'line', 'solid', "num_way", "pave"]
    keys = ['intersection', 'traffic_signals', 'lane', 'line', "num_way", 'solid']
    qs_list = []

    # ## About road intersection type
    if "intersection" in keys:
        qs_list += [gen_multi_exist_quesiton("intersection"),
                    gen_multi_exist_quesiton("t intersection"),
                    gen_multi_exist_quesiton("roundabout"),
                    gen_multi_exist_quesiton("crosswalk")]

    # ## About traffic signals
    if "traffic_signals" in keys:
        qs_list += [gen_multi_exist_quesiton("traffic sign"),
                    gen_multi_exist_quesiton("stop sign"),
                    gen_multi_exist_quesiton("speed limit sign"),
                    gen_multi_exist_quesiton("do no enter sign"),
                    gen_multi_exist_quesiton("no left turn sign"),
                    gen_multi_exist_quesiton("no right turn sign")]

    # ## About one-lane or two-lane
    if "lane" in keys:
        qs_list += [gen_multi_exist_quesiton("two lane road"),
                    gen_multi_exist_quesiton("one lane road")]

    # ## About 1-way or 2-way street or paved
    if "num_way" in keys:
        qs_list += [gen_multi_exist_quesiton("two way road"),
                    gen_multi_exist_quesiton("one way road")]

    # ## About whether paved
    if "pave" in keys:
        pass

    # ## About Solidity
    if "solid" in keys:
        qs_list += [gen_multi_exist_quesiton("unbroken line"),
                    gen_multi_exist_quesiton("broken line")]

    ## return road
    return qs_list


def gen_multi_exist_quesiton(item):
    """
    rule: the input traffic rule
    item: the component we want to know, e.g. intersection, roundabout, crosswalk, stop sign...
    return: a list of all the questions
    """
    return [
        gen_question(f"is there {item}"),
        gen_question(f"is it {item}"),
        gen_question(f"is {item} mentioned"),
        gen_question(f"is {item} explicitly mentioned"),
        gen_question(f"is this {item}"),
    ]


def gen_question(question):
    ## return f"Based on the description, \"{rule}\", {question}, answer with yes or no?"
    return f"{question}"


def write_qs_road(gen_q_file):
    q_list = parse_road_network(0)

    if os.path.exists(gen_q_file):
        os.remove(gen_q_file)

    ## write down the generated questions
    for q_cls in q_list:
        for q in q_cls:
            write_q(gen_q_file, q.strip())


def main_road():
    text_file_path = '../text_files/'
    gen_q_file = 'q_road.txt'
    saving_path = text_file_path + gen_q_file

    write_qs_road(saving_path)
    print('Done: generating questions: {}'.format(saving_path))

if __name__ == "__main__":
    main_road()


