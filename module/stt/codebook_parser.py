from pyphonetics import Soundex
from pyphonetics import FuzzySoundex
from pyphonetics import Metaphone
from pyphonetics import Lein
from pyphonetics import MatchingRatingApproach
from pyphonetics import RefinedSoundex
from pyphonetics.distance_metrics import levenshtein_distance as distance
import numpy as np
# import rospy
# import os
# import yaml


import re

soundex = Soundex()
fuzzy_soundex = FuzzySoundex()
metaphone = Metaphone()
lein = Lein()
mra = MatchingRatingApproach()
rsoundex = RefinedSoundex()

def parser(input, word_list=None):
    if len(input) == 0: return ""
    split = input.split('and') # 왜 and split? ex) my name is charlie and my favourite drink is cola 뭐 이런건가
    
    lst = word_list

    order_list =[]
    for item in split:
        if item == '': continue
        item = item.strip()
        item = item.lower()
        if item in lst:
            order_list.append(item)
            print('parser item in lst: ', item, distance(item, item))
            continue
        distmin = np.infty
        minstring = ''
        for i in lst:
            current = 2 * fuzzy_soundex.distance(i, item) + 0.2 * metaphone.distance(i, item) + 0.1 * lein.distance(i,item)
            if distmin > current:
                distmin = current
                minstring = i
            elif distmin == current:
                if distance(item, i) < distance(item, minstring):
                    minstring = i
        order_list.append(minstring)
        print('parser item, distmin: ', item, distmin)
        #rospy.loginfo(i, distmin)
    print('parser order_list', order_list)
    order = ""
    for i, item in enumerate(order_list):
        order += item
        if i < len(order_list)-1:
            order += " and "
    return order


def parser_single(input, word_list=None, normalize=False):
    if len(input) == 0: 
        return input, 0

    item = input.strip().lower()
    if item in word_list:
        return item, 0
    
    distmin = np.infty
    minstring = ''

    for word in word_list:
        current = 2 * fuzzy_soundex.distance(word, item) + 0.2 * metaphone.distance(word, item) + 0.1 * lein.distance(word, item)
        if normalize:
            current /= len(word)
        print(word, current)
        if distmin > current:
            distmin = current
            minstring = word
        elif distmin == current:
            if distance(item, word) < distance(item, minstring):
                minstring = word

    return minstring, distmin


def parser_sentence(input, mode=None, word_list=None):
    import spacy

    lower_input = input.strip().lower()

    sentence_parse_result = ''
    word_parse_result = ''

    candidate_remove_list = ['my', 'is', 'i', 'name', 'favorite', 'drink', 'it']
    
    patterns = []
    if mode == 'name':
        patterns = [
            r"my name is (\w+)[.,]?",
            r"i'm (\w+)[.,]?",
            r"i am (\w+)[.,]?",
            r"call me (\w+)[.,]?",
            r"(\w+) is my name[.,]?",
        ]
        label='PERSON'
    elif mode == 'drink':
        patterns = [
            r"my favorite drink is ([\w\s]+)[.,]?",
            r"my favourite drink is ([\w\s]+)[.,]?",
            r"i like ([\w\s]+)[.,]?",
            r"i prefer ([\w\s]+)[.,]?",
            r"([\w\s]+) is my favorite drink[.,]?",
            r"([\w\s]+) is my favourite drink[.,]?",
            r"([\w\s]+) is my favorite[.,]?",
            r"([\w\s]+) is my favourite[.,]?",
            r"it's ([\w\s]+)[.,]?",
            r"it is ([\w\s]+)[.,]?",
        ]
        label='PRODUCT'

    for pattern in patterns:
        print('pattern: ', pattern)
        match = re.search(pattern, lower_input, re.IGNORECASE)
        print('match: ', match)
        if match:
            print('match.group(1): ', match.group(1))
            sentence_parse_result = match.group(1).strip("., ")
            
    print('sentence_parse_result: ', sentence_parse_result)
    print()

    if sentence_parse_result == '':
        
        # spacy
        nlp = spacy.load("en_core_web_sm")
        doc = nlp(input)
        print('doc: ', doc)
        print('doc.ents: ', doc.ents)
        for ent in doc.ents:
            print('ent: ', ent)
            if ent.label_ == label:
                print('ent.text: ', ent.text)
                sentence_parse_result = ent.text
            
        if sentence_parse_result == '':
            sentence_split = list(lower_input.split())
            candidate_idx = []
            candidate_parse_list = []
            candidate_dist_list = []
            for i, w in enumerate(doc):
                print(w.text, w.pos_)
                if w.pos_ == 'NOUN' or w.pos_ == 'PROPN':
                    print(w.text)
                    if w.text in candidate_remove_list:
                        continue
                    # sentence_parse_result = w.text
                    candidate_idx.append(i)
            print('candidate_idx: ', candidate_idx)
            for i in candidate_idx:
                if i > len(sentence_split)-1:
                    continue
                print(sentence_split[i])
                parse, dist = parser_single(sentence_split[i], word_list, normalize=True)
                candidate_parse_list.append(parse)
                candidate_dist_list.append(dist)
                if i>0:
                    if sentence_split[i-1] not in candidate_remove_list:
                        print(sentence_split[i-1]+' '+sentence_split[i])
                        parse, dist = parser_single(sentence_split[i-1]+' '+sentence_split[i], word_list, normalize=True)
                        candidate_parse_list.append(parse)
                        candidate_dist_list.append(dist)
                if i<len(sentence_split)-1:
                    if sentence_split[i+1] not in candidate_remove_list:
                        print(sentence_split[i]+' '+sentence_split[i+1])
                        parse, dist = parser_single(sentence_split[i]+' '+sentence_split[i+1], word_list, normalize=True)
                        candidate_parse_list.append(parse)
                        candidate_dist_list.append(dist)
            print('candidate_parse_dist_list: ', list(zip(candidate_parse_list, candidate_dist_list)))
            if len(candidate_parse_list) == 0 or len(candidate_dist_list) == 0:
                sentence_parse_result, _ = parser_single(lower_input, word_list)
            else:
                sentence_parse_result = candidate_parse_list[np.argmin(candidate_dist_list)]

            if type(sentence_parse_result)==tuple:
                sentence_parse_result = sentence_parse_result[0]

            word_parse_result = sentence_parse_result
            print('word_parse_result: ', word_parse_result)
                                           
    print('sentence_parse_result: ', sentence_parse_result)
    print()

    if word_parse_result == '':
        word_parse_result = parser_single(sentence_parse_result, word_list)

    if type(word_parse_result)==tuple:
        word_parse_result = word_parse_result[0]
    
    return word_parse_result


'''

if __name__ == '__main__':
    dir = os.path.dirname(__file__)
    #print(k, success)
    #print(get_item_by_string('i want two sparkling water', ['Drinks']))
    #print(k, success)
    soundex_fail = 0
    fuzzy_fail = 0

    metaphone_fail = 0

    lein_fail = 0

    mra_fail = 0

    rsoundex_fail = 0
    with open(os.path.join(dir, 'codebook.yaml')) as f:
        codebook = yaml.load_all(f, Loader=yaml.FullLoader)
        pairlist = []
        for c in codebook:
            for object_type in c.keys():
                pairlist += list(c.values())[0]
        cnt = 0
        for word in pairlist:
            #print(list(word.keys())[0])
            for third in word.values():
                for i in third:
                    distmin = 10
                    minstring = ""
                    print("-------------------")
                    for word2 in pairlist:
                        for second in word2.keys():
                            #print(second)

                            current = 0.7*fuzzy_soundex.distance(i, second) + 0.2*metaphone.distance(i, second)+ 0.1*lein.distance(i, second)
                            if distmin > current:
                                distmin = current
                                minstring = second
                            elif distmin == current:
                                minstring += " " + second
                    #if minstring != list(word.keys())[0]:
                    #    if list(word.keys())[0] not in i:
                    if list(word.keys())[0] not in minstring:
                            print(f"{i},{minstring}, answer: {list(word.keys())[0]} ")
                            cnt +=1
        print(f"total mistake : {cnt}")




                            # print the minimum distance



                    # if not soundex.sounds_like(k, i):
                    # get minimum distance keyword

        # print(soundex.distance(k,i))
        '''

        # print(soundex_fail)
        ##print(fuzzy_fail)
        # print(metaphone_fail)
        # print(lein_fail)
        # print(mra_fail)
        # print(rsoundex_fail)




