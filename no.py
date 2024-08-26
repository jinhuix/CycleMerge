import numpy as np

def vector_to_permutation(vector):
    """
    将一个n维实值向量转化为对应的n个元素的排序。

    Args:
        vector (list): n维实值向量，每个元素的取值范围为[0, 1]。

    Returns:
        list: n个元素的排序，表示为整数列表，取值范围为[0, n-1]。
    """
    n = len(vector)
    
    vector = np.array(vector)
    
    # 根据扰动后的向量排序，得到排序索引
    ranking = np.argsort(vector).tolist()
    
    return ranking
print(vector_to_permutation([0,0,1]))