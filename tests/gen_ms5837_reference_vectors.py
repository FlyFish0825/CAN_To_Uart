"""生成 tests/ms5837_reference_vectors.h（跨实现参考向量表）。

为什么需要它：主机测试里还有一份测试自带的参考实现，但那份和固件同仓库、同语言、同一次编写，
容易一起错。这个脚本用**独立 Python**（无共享代码）按数据手册公式复算，覆盖
30BA/02BA、物理量程附近与极端输入、以及二阶温补的分支边界，用来抓常数、取整语义、
分支归属（例如 02BA 没有高温分支）这类错误。历史价值：本脚本第一版曾把 30BA 的高温分支
误用到 02BA 上，正是被这张表逐条比对抓出来的。

用法：python tests/gen_ms5837_reference_vectors.py
（随机种子固定，输出可复现；生成后直接提交 tests/ms5837_reference_vectors.h）
"""
import os
import random

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ms5837_reference_vectors.h')


def fd(a, b):
    """向负无穷取整（等价数据手册流程图里的 >> n）。"""
    return a // b


def comp(model, c, d1, d2):
    dT = d2 - c[5] * 256
    temp = 2000 + fd(dT * c[6], 8388608)
    if model == 30:
        off = c[2] * 65536 + fd(c[4] * dT, 128)
        sens = c[1] * 32768 + fd(c[3] * dT, 256)
    else:
        off = c[2] * 131072 + fd(c[4] * dT, 64)
        sens = c[1] * 65536 + fd(c[3] * dT, 128)
    ti = offi = sensi = 0
    if temp < 2000:
        if model == 30:
            ti = fd(3 * dT * dT, 8589934592)
            offi = fd(3 * (temp - 2000) ** 2, 2)
            sensi = fd(5 * (temp - 2000) ** 2, 8)
            if temp < -1500:
                offi += 7 * (temp + 1500) ** 2
                sensi += 4 * (temp + 1500) ** 2
        else:
            ti = fd(11 * dT * dT, 34359738368)
            offi = fd(31 * (temp - 2000) ** 2, 8)
            sensi = fd(63 * (temp - 2000) ** 2, 32)
    elif model == 30:
        # 只有 30BA 有高温分支；02BA 手册第 8 页没有高温项，Ti/OFFi/SENSi 保持 0。
        ti = fd(2 * dT * dT, 137438953472)
        offi = fd((temp - 2000) ** 2, 16)
        sensi = 0
    div = 8192 if model == 30 else 32768
    p = fd(fd(d1 * (sens - sensi), 2097152) - (off - offi), div)
    return p, temp - ti


def main():
    rng = random.Random(20261006)
    rows = []
    for model in (30, 2):
        vecs = []
        for _ in range(40):
            c = [0, rng.randint(20000, 65000), rng.randint(20000, 65000),
                 rng.randint(10000, 40000), rng.randint(10000, 40000),
                 rng.randint(25000, 35000), rng.randint(20000, 35000)]
            dt = rng.randint(-1200000, 1200000)
            d2 = max(1, min(0xFFFFFE, c[5] * 256 + dt))
            d1 = rng.randint(1, 0xFFFFFE)
            vecs.append((c, d1, d2))
        extremes = [
            ([0, 1, 1, 1, 1, 1, 1], 1, 1),
            ([0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF], 0xFFFFFE, 0xFFFFFE),
            ([0, 0xFFFF, 0x4E20, 0x9C40, 0x7530, 0x9C40, 0x7530], 1, 0xFFFFFE),
            ([0, 0x4E20, 0xFFFF, 0x2710, 0x4E20, 0x61A8, 0x61A8], 0xFFFFFE, 1),
        ]
        for c, d1, d2 in extremes:
            vecs.append((c, d1, d2))
        base_c = [0, 40000, 36000, 20000, 22000, 26646, 26146]
        for target in (1999, 2000, 2001, -1499, -1500, -1501):
            lo, hi = -16000000, 16000000
            while lo < hi:
                mid = (lo + hi) // 2
                if 2000 + fd(mid * base_c[6], 8388608) < target:
                    lo = mid + 1
                else:
                    hi = mid
            d2 = max(1, min(0xFFFFFE, base_c[5] * 256 + lo))
            vecs.append((base_c, 4958179 if model == 30 else 6465444, d2))
        for c, d1, d2 in vecs:
            p, t = comp(model, c, d1, d2)
            rows.append((model, c, d1, d2, p, t))

    with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
        f.write('/*\n')
        f.write(' * MS5837 参考向量表（自动生成，勿手改）。\n *\n')
        f.write(' * 生成脚本：tests/gen_ms5837_reference_vectors.py（独立 Python 实现，随机种子 20261006）。\n')
        f.write(' * 每行：model / C1..C6 / D1 / D2 / 期望整数压力 / 期望 0.01°C 温度。\n')
        f.write(' * 用途：交叉实现校验 Ms5837_Compensate() 的常数、取整语义与二阶分支归属。\n */\n')
        f.write('#ifndef __MS5837_REFERENCE_VECTORS_H__\n#define __MS5837_REFERENCE_VECTORS_H__\n\n')
        f.write('typedef struct\n{\n  uint8_t model;\n  uint16_t coefficient[7]; /* [0] 占位，[1..6] = C1..C6 */\n'
                '  uint32_t d1;\n  uint32_t d2;\n  int64_t expected_raw;\n'
                '  int32_t expected_temp_centi_c;\n} Ms5837ReferenceVector_t;\n\n')
        f.write('static const Ms5837ReferenceVector_t ms5837_reference_vectors[] =\n{\n')
        for model, c, d1, d2, p, t in rows:
            f.write('  {%2dU, {0U, %5dU, %5dU, %5dU, %5dU, %5dU, %5dU}, %8dU, %8dU, %9dLL, %6d},\n'
                    % (model, c[1], c[2], c[3], c[4], c[5], c[6], d1, d2, p, t))
        f.write('};\n\n')
        f.write('#define MS5837_REFERENCE_VECTOR_COUNT '
                '(sizeof(ms5837_reference_vectors) / sizeof(ms5837_reference_vectors[0]))\n\n')
        f.write('#endif /* __MS5837_REFERENCE_VECTORS_H__ */\n')
    print('vectors: %d -> %s' % (len(rows), OUT))


if __name__ == '__main__':
    main()
