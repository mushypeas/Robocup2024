from utils.gpsr_configs import *

def make_dict(command_type_idx, command):
    command_dict = {}
    specific_words = []
    for word in command:
        if word not in command_type[command_type_idx]:
            specific_words.append([word])


##################################################################################
# type 0, type 1, type 2, type 3, type 5, type 6, type 7, type 8, type 12, type 13
##################################################################################
    if command_type_idx in [0, 1, 2, 3, 5, 6, 7, 8, 11, 12, 13]:
        target_pl = []
        target_obj = []
        target_cat = []
        ref = []
        ref_prop = []
        prop = []
        obj_pl = []
        person = []

        places = []  # rooms or locations
        for s_word in specific_words:
            if s_word in objects:
                if 'object' in command:
                    if s_word in objects_sub_words:
                        ref += s_word
                    else:
                        ref = s_word
                else:
                    if s_word in objects_sub_words:
                        target_obj += s_word
                    else:
                        target_obj = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)
            elif s_word in male_names or s_word in female_names:
                person = s_word
            elif s_word in object_types:
                if s_word in ['cleaning', 'supplies', 'supply']:
                    target_cat = ['cleaning supplies']
                else:
                    target_cat = s_word

        person = []
        if 'person' in command:
            for idx, word in enumerate(command):
                if word == 'person':
                    person_idx = idx
            person.append('person')
            for idx, word in enumerate(command):
                if word == 'in':
                    break
                elif idx > person_idx:
                    person.append(word)

        if 'object' in command:
            if len(ref) == 0:
                obj_idx = command.index('object')
                prop = [command[obj_idx - 1]]
                if command[obj_idx - 2] in ['right', 'left']:
                    prop.insert(0, command[obj_idx - 2])
            else:
                ref_idx = command.index(ref[0])
                obj_idx = command.index('object')
                ref_prop += (command[obj_idx + 1: ref_idx - 1])

        if len(target_cat) != 0:
            cat_idx = command.index(target_cat[0])
            prop.append(command[cat_idx - 1])

        if ('from' in command) or ('go' in command) or ('navigate' in command):
            if len(places) == 1:
                obj_pl = places[0]
                target_pl = ['here']
            elif len(places) == 2:
                if places[1] in rooms_sub_words or places[1] in locations_sub_words:  # places[1][0]
                    obj_pl += places[0] + places[1]
                    target_pl = ['here']
                else:
                    obj_pl = places[0]
                    target_pl = places[1]
            elif len(places) == 3:
                if places[1] in rooms_sub_words or places[1] in locations_sub_words:  # places[1][0]
                    obj_pl += places[0] + places[1]
                    target_pl = places[2]
                else:
                    obj_pl = places[0]
                    target_pl += places[1] + places[2]
            elif len(places) == 4:
                obj_pl += places[0] + places[1]
                target_pl += places[2] + places[3]
        else:
            obj_pl = ['here']
            if len(places) == 1:
                target_pl = places[0]
            elif len(places) == 2:
                target_pl += places[0] + places[1]

        command_dict['action'] = 'bring'
        command_dict['target_pl'] = ' '.join(target_pl)
        command_dict['target_obj'] = ' '.join(target_obj)
        command_dict['target_cat'] = ' '.join(target_cat)
        command_dict['ref'] = ' '.join(ref)
        command_dict['ref_prop'] = ' '.join(ref_prop)
        command_dict['prop'] = ' '.join(prop)
        command_dict['obj_pl'] = ' '.join(obj_pl)
        command_dict['person'] = ' '.join(person)


#########################################################################
# type 4
#########################################################################
    elif command_type_idx == 4:
        command_dict['action'] = 'bring_bag'
        places = []
        for s_word in specific_words:
            if s_word in male_names or s_word in female_names:
                person = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)
        if len(places) == 1:
            target_pl = places[0]
        else:
            target_pl = places[0] + places[1]
        command_dict['target_pl'] = ' '.join(target_pl)



#########################################################################
# type 9
#########################################################################
    elif command_type_idx == 9:
        pass


#########################################################################
# type 10
#########################################################################
    elif command_type_idx == 10:
        command_dict['action'] = 'take_out_trash'


#########################################################################
# type 14, type 15, type 16
# meet & follow
#########################################################################
    elif command_type_idx in [14, 15, 16]:
        places = []
        for s_word in specific_words:
            if s_word in male_names or s_word in female_names:
                person = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)

        if len(places) == 2:
            first_place = places[0]
            final_place = places[1]
        elif len(places) == 3:
            if places[1] in rooms_sub_words or places[1] in locations_sub_words:
                first_place = places[0] + places[1]
                final_place = places[2]
            else:
                first_place = places[0]
                final_place = places[1] + places[2]
        else:
            first_place = places[0] + places[1]
            final_place = places[2] + places[3]

        command_dict['action'] = 'follow'
        command_dict['first_place'] = ' '.join(first_place)
        command_dict['final_place'] = ' '.join(final_place)
        command_dict['person'] = ' '.join(person)


#########################################################################
# type 17, type 18, type 19, type 28, type 29
# meet & guide
#########################################################################
    elif command_type_idx in [17, 18, 19, 28, 29]:
        places = []
        for s_word in specific_words:
            if s_word in male_names or s_word in female_names:
                person = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)
        for word in command:
            if word in ['taxi', 'cab', 'uber']:
                places.append([word])
        print(places)

        if len(places) == 2:
            first_place = places[0]
            final_place = places[1]
        elif len(places) == 3:
            if places[1] in rooms_sub_words or places[1] in locations_sub_words:
                first_place = places[0] + places[1]
                final_place = places[2]
            else:
                first_place = places[0]
                final_place = places[1] + places[2]
        else:
            first_place = places[0] + places[1]
            final_place = places[2] + places[3]

        command_dict['action'] = 'guide'
        command_dict['first_place'] = ' '.join(first_place)
        command_dict['final_place'] = ' '.join(final_place)
        command_dict['person'] = ' '.join(person)


#########################################################################
# type 20
# tell name
#########################################################################
    elif command_type_idx == 20:
        places = []
        for s_word in specific_words:
            if s_word in rooms or s_word in locations:
                places.append(s_word)

        if len(places) == 1:
            first_place = places[0]
        else:
            first_place = places[0] + places[1]

        command_dict['action'] = 'tell_name'
        command_dict['first_place'] = ' '.join(first_place)


#########################################################################
# type 25, type 27
# meet & introduce
#########################################################################
    elif command_type_idx == 25 or command_type_idx == 27:
        places = []
        person = []
        for s_word in specific_words:
            if s_word in male_names or s_word in female_names:
                if len(person) != 0:
                    continue
                person = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)

        if len(places) == 2:
            first_place = places[0]
            final_place = places[1]
        elif len(places) == 3:
            if places[1] in rooms_sub_words or places[1] in locations_sub_words:
                first_place = places[0] + places[1]
                final_place = places[2]
            else:
                first_place = places[0]
                final_place = places[1] + places[2]
        else:
            first_place = places[0] + places[1]
            final_place = places[2] + places[3]

        subject = []
        for idx, word in enumerate(command):
            if word == 'to':
                idx_of_to = idx
        for idx, word in enumerate(command):
            if word == 'in':
                break
            elif word == 'at' and idx > idx_of_to:
                break
            elif idx > idx_of_to:
                subject += [word]

        command_dict['action'] = 'introduce'
        command_dict['first_place'] = ' '.join(first_place)
        command_dict['final_place'] = ' '.join(final_place)
        command_dict['person'] = ' '.join(person)
        command_dict['subject'] = ' '.join(subject)


#########################################################################
# type 26
# meet & ask
#########################################################################
    elif command_type_idx == 26:
        places = []
        for s_word in specific_words:
            if s_word in male_names or s_word in female_names:
                person = s_word
            elif s_word in rooms or s_word in locations:
                places.append(s_word)

        if len(places) == 1:
            first_place = places[0]
        else:
            first_place = places[0] + places[1]

        command_dict['action'] = 'ask_to_leave'
        command_dict['first_place'] = ' '.join(first_place)
        command_dict['person'] = ' '.join(person)


#########################################################################
# type 30, type 31, type 32
# meet & say
#########################################################################
    elif command_type_idx in [30, 31, 32]:
        places = []
        for s_word in specific_words:
            if s_word in rooms or s_word in locations:
                places += s_word

        subject = []
        if 'person' in command:
            for idx, word in enumerate(command):
                if word == 'person':
                    person_idx = idx
            subject.append('person')
            for idx, word in enumerate(command):
                if word == 'in':
                    break
                elif idx > person_idx:
                    subject.append(word)
        else:
            for word in command:
                if word in ['man', 'woman', 'boy', 'girl', 'male', 'female']:
                    subject.append(word)

        if 'something' in command:
            command_dict['action'] = '0'  # something about yourself
        elif 'time' in command:
            command_dict['action'] = '1'  # time
        elif 'today' in command:
            command_dict['action'] = '2'  # what day is today
        elif 'tomorrow' in command:
            command_dict['action'] = '3'  # what day is tomorrow
        elif 'name' in command:
            command_dict['action'] = '4'  # your team's name
        elif 'country' in command:
            command_dict['action'] = '5'  # your team's country
        elif 'affiliation' in command:
            command_dict['action'] = '6'  # your team's affiliation
        elif 'week' in command:
            command_dict['action'] = '7'  # the day of the week
        elif 'month' in command:
            command_dict['action'] = '8'  # the day of the month
        elif 'joke' in command:
            command_dict['action'] = '9'  # a joke
        elif 'answer' in command:
            command_dict['action'] = '10'  # answer a question

        command_dict['subject'] = ' '.join(subject)
        command_dict['first_place'] = ' '.join(places)
        command_dict['full_command'] = command


#########################################################################
# type 33, type 34, type 35, type 36, type 37
# how many & find & what prop & what three prop $ find three
#########################################################################
    elif command_type_idx in [33, 34, 35, 36, 37]:
        target_obj = []
        target_cat = []
        places = []
        for s_word in specific_words:
            if s_word in objects or s_word in objects_sub_words:
                target_obj += s_word
            elif s_word in object_types:
                target_cat += s_word
            elif s_word in rooms or s_word in locations:
                places += s_word

        prop = []
        for word in command:
            if word in ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']:
                prop.append(word)

        if 'many' in command:
            command_dict['action'] = 'how_many'
        elif len(prop) != 0:
            if 'three' in command:
                command_dict['action'] = 'three_what_prop'
            else:
                command_dict['action'] = 'what_prop'
        else:
            if 'three' in command:
                command_dict['action'] = 'find_three'
            else:
                command_dict['action'] = 'find'


        command_dict['target_obj'] = ' '.join(target_obj)
        command_dict['target_cat'] = ' '.join(target_cat)
        command_dict['obj_pl'] = ' '.join(places)
        command_dict['prop'] = ' '.join(prop)

    return command_dict