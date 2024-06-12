from pyphonetics import Soundex
from pyphonetics import FuzzySoundex
from pyphonetics import Metaphone
from pyphonetics import Lein
from pyphonetics import MatchingRatingApproach
from pyphonetics import RefinedSoundex
from pyphonetics.distance_metrics import levenshtein_distance as distance
import numpy as np
import rospy
import os
import yaml

soundex = Soundex()
fuzzy_soundex = FuzzySoundex()
metaphone = Metaphone()
lein = Lein()
mra = MatchingRatingApproach()
rsoundex = RefinedSoundex()

def parser(input, word_list=None):
    if len(input) == 0: return ""
    split = input.split('and') # 왜 and split? ex) my name is charlie and my favourite drink is cola 뭐 이런건가
    
    # lst = ['red wine', 'cola','juice pack','tropical juice','milk','iced tea','orange juice','tuna','tomato soup','spam','mustard','strawberry jello','chocolate jello','coffee grounds','sugar','pear','plum','peach','lemon','orange','strawberry','banana','apple','pringles','cornflakes','cheezit', 'yes','no']
    # lst.extend(['adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone'])
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




