from write import *


def gen_multi_be_qs(item):
    if item != 'vehicle':
        return [
            gen_ego_basic_question(f"there is a {item}"),
            gen_ego_basic_question(f"it is a {item}"),
            gen_ego_basic_question(f"a {item} is mentioned"),
            gen_ego_basic_question(f"a {item} is explicitly mentioned"),
            gen_ego_basic_question(f"you can see a {item}"),
        ]
    else:
        return [
            gen_ego_basic_question(f"there is another {item}"),
            gen_ego_basic_question(f"it is another {item}"),
            gen_ego_basic_question(f"another {item} is mentioned"),
            gen_ego_basic_question(f"another {item} is explicitly mentioned"),
            gen_ego_basic_question(f"you can see another {item}"),
        ]

def gen_ego_basic_question(question):
    ## return f"Based on the description, \"{rule}\", {question}, answer with yes or no?"
    return f"Is it possible that {question}"


def gen_multi_do_qs(item):
    return [
        gen_ego_basic_question(f"the driver is {item}"),
        gen_ego_basic_question(f"I am {item}"),
        gen_ego_basic_question(f"you are {item}"),
        gen_ego_basic_question(f"the car is {item}"),
        gen_ego_basic_question(f"the ego_vehicle is {item}"),
    ]


def gen_multi_can_qs(item):
    return [
        gen_ego_basic_question(f"the driver can {item}"),
        gen_ego_basic_question(f"I can {item}"),
        gen_ego_basic_question(f"you can {item}"),
        gen_ego_basic_question(f"the {item} can be seen by the driver"),
        gen_ego_basic_question(f"the {item} can be seen by you"),
    ]


def gen_other_actor_exist_qs():
    qs_list = []
    item_list = ["vehicle", "pedestrian"]
    for item in item_list:
        qs_list.append(gen_multi_be_qs(item))

    return qs_list


def gen_other_actor_behavior_qs(other_actor):
    def gen_multi_do_qs(other_actor, item):
        return [
            gen_ego_basic_question(f"the driver can see a {other_actor} {item}"),
            gen_ego_basic_question(f"I can see a {other_actor} {item}"),
            gen_ego_basic_question(f"you can see a {other_actor} {item}"),
            gen_ego_basic_question(f"there is another {other_actor} other than me {item}"),
            gen_ego_basic_question(f"there is another {other_actor} besides me {item}"),
        ]

    qs_list = []
    item_list = ["going along the road",
                 "static",
                 "turning left", "turning right",
                 "changing lane", "changing lane to left", "changing lane to right"]
    for item in item_list:
        qs_list.append(gen_multi_do_qs(other_actor, item))

    return qs_list


def gen_other_actor_position_qs(other_actor, road_element):
    def gen_multi_position_qs(other_actor, item):
        return [
            gen_ego_basic_question(f"there is one {other_actor} {item}"),
            gen_ego_basic_question(f"there is a {other_actor} {item}"),
            gen_ego_basic_question(f"you can see a {other_actor} {item}"),
            gen_ego_basic_question(f"another {other_actor} {item}"),
            gen_ego_basic_question(f"there is another {other_actor} {item}"),
        ]

    qs_list = []
    if road_element is not None:
        item_list = [f"in the {road_element[0]}",
                     f"at the left of the {road_element[0]}",
                     f"at the right of the {road_element[0]}",
                     f"at the opposite of the {road_element[0]}"]

        for item in item_list:
            qs_list.append(gen_multi_position_qs(other_actor, item))



    if other_actor != 'pedestrian':
        item_list = [f"in the front of you",
                     f"in the behind of you"]
        for item in item_list:
            qs_list.append(gen_multi_position_qs(item))


    return qs_list


def gen_ego_position_qs(road_element):
    qs_list = []
    if road_element in ['crosswalk', 'intersection', 'roundabout']:
        qs_list.append(gen_multi_do_qs('in the {}'.format(road_element)))
        qs_list.append(gen_multi_do_qs('driving towards the {}'.format(road_element)))
    elif 'sign' in road_element:
        qs_list.append(gen_multi_can_qs('see the'.format(road_element)))

    return qs_list




def gen_ego_behavior_qs():


    qs_list = []
    item_list = ["traveling along the road",
                 "static",
                 "turning left", "turning right",
                 "changing lane", "changing lane to left", "changing lane to right"]
    for item in item_list:
        qs_list.append(gen_multi_do_qs(item))



def gen_ego_qs():
    """ ego vehicle Questions """
    keys = ["ego_behavior", "ego_position"]
    qs_list = []

    def gen_multi_do_qs(item):
        return [
            gen_ego_basic_question(f"the driver is {item}"),
            gen_ego_basic_question(f"I am {item}"),
            gen_ego_basic_question(f"you are {item}"),
            gen_ego_basic_question(f"the car is {item}"),
            gen_ego_basic_question(f"the ego_vehicle is {item}"),
        ]

    def gen_multi_can_qs(item):
        return [
            gen_ego_basic_question(f"the driver can {item}"),
            gen_ego_basic_question(f"I can {item}"),
            gen_ego_basic_question(f"you can {item}"),
            gen_ego_basic_question(f"the {item} can be seen by the driver"),
            gen_ego_basic_question(f"the {item} can be seen by you"),
        ]

    # ## About ego_vehicle's behaviour
    if "ego_behavior" in keys:
        item_list = ["traveling along the road",
                     "static",
                     "turning left", "turning right",
                     "changing lane", "changing lane to left", "changing lane to right"]
        for item in item_list:
            qs_list.append(gen_multi_do_qs(item))


    # ## About ego_vehicle's position
    if "ego_position" in keys:
        # in different kind of road networks
        item_list = ["in the roundabout", "driving towards the roundabout",
                     "in the intersection", "driving towards the intersection",
                     "in the crosswalk", "driving towards the crosswalk"]
        for item in item_list:
            qs_list.append(gen_multi_do_qs(item))

        # whether can see various signs
        item_list = ["see the speed limit sign"]
        for item in item_list:
            qs_list.append(gen_multi_can_qs(item))

    return qs_list


def gen_other_actor_qs(other_actor, road):
    """ ego vehicle Questions """
    keys = ["other_actor_exist"]
    qs_list = []

    def gen_multi_be_qs(item):
        if item != 'vehicle':
            return [
                gen_ego_basic_question(f"there is a {item}"),
                gen_ego_basic_question(f"it is a {item}"),
                gen_ego_basic_question(f"a {item} is mentioned"),
                gen_ego_basic_question(f"a {item} is explicitly mentioned"),
                gen_ego_basic_question(f"you can see a {item}"),
            ]
        else:
            return [
                gen_ego_basic_question(f"there is another {item}"),
                gen_ego_basic_question(f"it is another {item}"),
                gen_ego_basic_question(f"another {item} is mentioned"),
                gen_ego_basic_question(f"another {item} is explicitly mentioned"),
                gen_ego_basic_question(f"you can see another {item}"),
            ]


    def gen_multi_do_qs(item):
        return [
            gen_ego_basic_question(f"the driver can see a {other_actor} {item}"),
            gen_ego_basic_question(f"I can see a {other_actor} {item}"),
            gen_ego_basic_question(f"you can see a {other_actor} {item}"),
            gen_ego_basic_question(f"there is another {other_actor} other than me {item}"),
            gen_ego_basic_question(f"there is another {other_actor} besides me {item}"),
        ]

    def gen_multi_position_qs(item):
        return [
            gen_ego_basic_question(f"there is one {other_actor} {item}"),
            gen_ego_basic_question(f"there is a {other_actor} {item}"),
            gen_ego_basic_question(f"you can see a {other_actor} {item}"),
            gen_ego_basic_question(f"another {other_actor} {item}"),
            gen_ego_basic_question(f"there is another {other_actor} {item}"),
        ]


    # ## About whether there is another actor
    if "other_actor_exist" in keys:
        item_list = ["vehicle", "pedestrian", "ambulance", "fire truck", "school bus"]
        for item in item_list:
            qs_list.append(gen_multi_be_qs(item))


    ## About another_vehicle's behaviour
    if other_actor:
        item_list = ["going along the road",
                     "static",
                     "turning left", "turning right",
                     "changing lane", "changing lane to left", "changing lane to right"]
        for item in item_list:
            qs_list.append(gen_multi_do_qs(item))

        # ask other actor's position
        if road:
            road = road[0]
            item_list = [f"in the {road}",
                         f"at the left of the {road}",
                         f"at the right of the {road}",
                         f"at the opposite of the {road}"]
            for item in item_list:
                qs_list.append(gen_multi_position_qs(item))
        else:
            qs_list.append([[''] * 5] * 4)


        if other_actor != 'pedestrian':
            item_list = [f"in the front of you",
                         f"in the behind of you"]
            for item in item_list:
                qs_list.append(gen_multi_position_qs(item))
        else:
            qs_list.append([[''] * 5] * 2)

    return qs_list






def write_qs_road(gen_q_file, q_list):
    if os.path.exists(gen_q_file):
        os.remove(gen_q_file)

    ## write down the generated questions
    for q_cls in q_list:
        for q in q_cls:
            write_q(gen_q_file, q.strip())

def main_ego():
    text_file_path = '../gpt3_ans/'
    gen_q_file = 'q_ego.txt'
    saving_path = text_file_path + gen_q_file

    # q_ego_list = gen_ego_qs()
    q_ego_list = gen_other_actor_qs()
    write_qs_road(saving_path, q_ego_list)
    print('Done: generating questions: {}'.format(saving_path))


if __name__ == "__main__":
    main_ego()




