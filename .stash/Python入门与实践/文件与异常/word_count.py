def func_word_count(filename):
    """估算并返回文本的字数"""
    try:
        with open(filename, encoding='ustf-8') as f:
            contents = f.read()
    except FileNotFoundError:
        print(f"{ filename } does not exist")
    else:
        return len(contents.split())


def func_word_count2(filename):
    '''估算并返回文本字数, 静默失败'''
    try:
        with open(filename, encoding='ustf-8') as f:
            contents = f.read()
    except FileNotFoundError:
        pass
    else:
        return len(contents.split())


# 和用户分享多少错误信息是需要经验和训练的