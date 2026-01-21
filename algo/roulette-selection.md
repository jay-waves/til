基于权重的概率选择算法, 即10权重个体被选中概率是1权重个体的10x.

```
for all members of population
    sum += fitness of this individual
end for

for all members of population
    probability = sum of probabilities + (fitness / sum)
    sum of probabilities += probability
end for

loop until new population is full
    do this twice
        number = Random between 0 and 1
        for all members of population
            if number > probability but less than next probability 
                then you have been selected
        end for
    end
    create offspring
end loop
```

> [roulette selection](https://stackoverflow.com/questions/177271/roulette-selection-in-genetic-algorithms)

概率算法, O(1):

> [Roulette-wheel selection via stochastic acceptance](http://lipowski.home.amu.edu.pl/homepage/roulette.html)
> 
> paper: [arxiv 1109.3627](../paper/Roulette-wheel%20selection%20via%20stochastic%20acceptance.pdf)

论文里测试不全, 权重分布不够平均时, 选择次数的期望会劣化. (虽然最劣化不会超过N)

然后没有考虑 random 多次选择的开销