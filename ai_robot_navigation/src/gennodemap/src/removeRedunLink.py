#!/usr/bin/env python3
# coding=utf-8

import time
# just a program to remove some links in origin output node map.
# not necessary to run this.

# cal distance^2
def calDist2(p1, p2):
    return ((p1['pos'][0]-p2['pos'][0])**2 + (p1['pos'][1]-p2['pos'][1])**2 + (p1['pos'][2]-p2['pos'][2])**2)

## in fact, no need to del all longest link
## if the longest link is < 0.8*sum_of_other
## link, it can be kept.
def findlongestlink(nodes, k1, k2, k3):
    di = {'1':{'links':[k1,k2],'dist':nodes[k1]['link'][k2]}, '2':{'links':[k1,k3],'dist':nodes[k1]['link'][k3]}, '3':{'links':[k2,k3],'dist':nodes[k2]['link'][k3]}}

    rate = 0.99
    if(di['1']['dist'] > rate*(di['2']['dist']+di['3']['dist'])):
        longestlinks = di['1']['links']
    elif(di['2']['dist'] > rate*(di['1']['dist']+di['3']['dist'])):
        longestlinks = di['2']['links']
    elif(di['3']['dist'] > rate*(di['1']['dist']+di['2']['dist'])):
        longestlinks = di['3']['links']
    else:
        longestlinks = None
    '''
    if(di['1']['dist'] > di['2']['dist']):
        if(di['1']['dist'] > di['3']['dist']):
            longestlinks = di['1']['links']
        else:
            longestlinks = di['3']['links']
    else:
        if(di['2']['dist'] > di['3']['dist']):
            longestlinks = di['2']['links']
        else:
            longestlinks = di['3']['links']
    '''
    print(di, longestlinks)

    return longestlinks

def delalllinks(nodes, dellinks):
    for links in dellinks:
        #print(links)
        #print(type(nodes[links[0]]['link'][links[1]]))#
        if(links==None):
            continue
        if(len(nodes[links[0]]['link']) > 2 and len(nodes[links[1]]['link']) > 2 and links[1] in nodes[links[0]]['link'] and links[0] in nodes[links[1]]['link']):
            del nodes[links[0]]['link'][links[1]]
            del nodes[links[1]]['link'][links[0]]

if __name__ == '__main__':
    f1 = open('Result/denKeyfPos.txt','r')
    f2 = open('Result/denKeyfPosRelation.txt','r')

    nodes = {}
    keyposlines = f1.readlines()
    i = 0
    for line in keyposlines:
        #print(line)
        x,y,z = line.split(' ')
        x = float(x)
        y = float(y)
        z = float(z)
        #print(x,y,z)
        nodes[str(i)] = {'pos':[x,y,z], 'link':{}}
        i += 1

    linklines = f2.readlines()
    for line in linklines:
        a,b = line.split(' ')
        b = str(int(b)) #remove \n
        dist = calDist2(nodes[a], nodes[b])
        nodes[a]['link'][b] = dist
        nodes[b]['link'][a] = dist
    #print(nodes)

    f1.close()
    f2.close()
    st = time.clock()
    # remove redundant link
    for ki in nodes.keys():
        if(len(nodes[ki]['link']) > 2):
            dellinksid = []
            for kj in nodes[ki]['link'].keys():
                for kk in nodes[ki]['link'].keys():
                    if(kk == kj):
                        continue
                    else:
                        if(kk in nodes[kj]['link'].keys()):
                            #remove the longest link
                            dellink = findlongestlink(nodes, ki, kj, kk)
                            dellinksid.append(dellink)
            #print(dellinksid)
            delalllinks(nodes, dellinksid)
    tt = time.clock() - st
    print(tt)

    f3 = open('Result/denKeyfPosRelation1.txt','w')
    for ki in nodes.keys():
        for kj in nodes[ki]['link'].keys():
            if(int(ki)<int(kj)):
                f3.write(ki+' '+kj+'\n')
    f3.close()
    
