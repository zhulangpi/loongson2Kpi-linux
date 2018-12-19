/*
 * IEEE754 floating point arithmetic
 * double precision: MADDF.f (Fused Multiply Add)
 * MADDF.fmt: FPR[fd] = FPR[fr] + (FPR[fs] x FPR[ft])
 *
 * MIPS floating point support
 * Copyright (C) 2015 Imagination Technologies, Ltd.
 * Author: Markos Chandras <markos.chandras@imgtec.com>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; version 2 of the License.
 */

#include "ieee754dp.h"
#define UINT64_C(x) x##ULL
#define DP_FBITS 52

#if defined (CONFIG_CPU_LOONGSON3) || defined (CONFIG_CPU_MIPS64R2) || defined(CONFIG_CPU_LOONGSON2K)
inline int dclz(unsigned long int a0)
{
	int cnt;
	asm (
		"dclz\t%[cnt], %[a0]\n\t"
		: [cnt] "=&r" (cnt)
		: [a0] "r" (a0));

	return cnt;
}
#else
inline int dclz(unsigned long int a0)
{
	int i = 0;

	while (i < 64) {
		if (a0 & 0x8000000000000000)
			return i;
		a0 <<= 1;
		i++;
	}
	return i;
}
#endif

static ieee754dp _ieee754dp_maddf(ieee754dp z, ieee754dp x,
				ieee754dp y, int opt)
{
	int re;
	int rs;
	u64 rm = 0;
	unsigned lxm;
	unsigned hxm;
	unsigned lym;
	unsigned hym;
	u64 lrm;
	u64 hrm;
	u64 t;
	u64 at;
	int s;

	COMPXDP;
	COMPYDP;

	u64 zm; int ze; int zs __maybe_unused; int zc;
	u64 lzm = 0;
	u64 hzm = 0;

	EXPLODEXDP;
	EXPLODEYDP;
	EXPLODEDP(z, zc, zs, ze, zm)

	FLUSHXDP;
	FLUSHYDP;
	FLUSHDP(z, zc, zs, ze, zm);

	CLEARCX;

	switch (zc) {
	case IEEE754_CLASS_SNAN:
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754dp_nanxcpt(z, "maddf.d");
	case IEEE754_CLASS_DNORM:
		DPDNORMx(zm, ze);
	/* QNAN is handled separately below */
	}

	switch (CLPAIR(xc, yc)) {
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_SNAN):
		return ieee754dp_nanxcpt(y, "maddf.d");

	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_INF):
		return ieee754dp_nanxcpt(x, "maddf.d");

	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_QNAN):
		return y;

	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_INF):
		return x;


	/*
	 * Infinity handling
	 */
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_INF):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754dp_indef();

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_INF):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		return ieee754dp_inf(xs ^ ys);

	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_ZERO):
		if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		/* Multiplication is 0 so just return z */
		return z;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_DNORM):
		DPDNORMX;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_DNORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		else if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		DPDNORMY;
		break;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		else if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		DPDNORMX;
		break;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		else if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		/* fall through to real computations */
	}

	/* Finally get to do some computation */

	/*
	 * Do the multiplication bit first
	 *
	 * rm = xm * ym, re = xe + ye basically
	 *
	 * At this point xm and ym should have been normalized.
	 */
	assert(xm & DP_HIDDEN_BIT);
	assert(ym & DP_HIDDEN_BIT);

	re = xe + ye;
#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K)
	rs = xs ^ ys ^ (opt & 2);
	zs = zs ^ (opt & 1) ^ (opt & 2);
	xm <<= 64 - (DP_FBITS + 2);
	ym <<= 64 - (DP_FBITS + 2);
#else
	rs = xs ^ ys;

	/* shunt to top of word */
	xm <<= 64 - (DP_FBITS + 1);
	ym <<= 64 - (DP_FBITS + 1);
#endif
	/*
	 * Multiply 32 bits xm, ym to give high 32 bits rm with stickness.
	 */

	/* 32 * 32 => 64 */
#define DPXMULT(x, y)	((u64)(x) * (u64)y)

	lxm = xm;
	hxm = xm >> 32;
	lym = ym;
	hym = ym >> 32;

	lrm = DPXMULT(lxm, lym);
	hrm = DPXMULT(hxm, hym);

	t = DPXMULT(lxm, hym);

	at = lrm + (t << 32);
	hrm += at < lrm;
	lrm = at;

	hrm = hrm + (t >> 32);

	t = DPXMULT(hxm, lym);

	at = lrm + (t << 32);
	hrm += at < lrm;
	lrm = at;

	hrm = hrm + (t >> 32);
#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K)
	/* here we need to normalize hrm|rm */
	if (hrm < UINT64_C( 0x2000000000000000)) {
		hrm = (hrm << 1) | ((lrm & 0x8000000000000000) != 0);
		lrm =  lrm << 1;
	} else
		re++;

	rm = hrm | (lrm != 0);

	if (zc == IEEE754_CLASS_ZERO)
		goto done;


	zm = (zm << 9);
	/* do the 106/105bit + 53bit  */
	/* adjust mantissa and exponent */
	s = re - ze;
	if (s < 0) {
		re = ze;
		if ((zs == rs) || (s < -1))
			hrm = (-s) < 63 ? hrm >> (-s) | ((uint64_t) (hrm << (s & 63)) != 0) :
				(hrm != 0);
		else {
			lrm = hrm << 63 | lrm >> 1 | ((uint64_t) (lrm << 63) != 0);
			hrm = hrm >> 1;
		}
	} else if (s > 0) {
		if (s < 64) {
			hzm = zm >> s;
			lzm = zm << (-s & 63);
		} else {
			hzm = 0;
			lzm = (s < 127) ? zm >> (s & 63) |
				((zm & (((uint64_t) 1 << ( s & 63)) - 1)) != 0) :
				(zm != 0);
		}
	}


	if (zs == rs) {
		if (s <= 0) {
			rm = hrm = (hrm + zm) | (lrm != 0);
		} else {
			lrm = at = lrm + lzm;
			hrm = hrm + hzm + (at < lzm);
			rm = hrm | (lrm != 0);
		}

		if (rm < UINT64_C(0x4000000000000000))
			rm <<= 1;
		else
			re++;

	} else {
		if (s < 0) {
			rs = zs;
			hrm = zm - hrm - (lrm != 0);
			lrm = -lrm;
		} else if (!s) {
			hrm = hrm - zm;
			if (!(hrm | lrm))
				return ieee754dp_zero(ieee754_csr.rm == FPU_CSR_RD);
			if (hrm & UINT64_C(0x8000000000000000)) {
				rs = !rs;
				hrm = -hrm - (lrm != 0);
				lrm = -lrm;
			}
		} else {
			at =  (lrm < lzm);
			lrm = lrm - lzm;
			hrm = hrm - hzm - at;
		}

		/* adjust the iff if rs != zs */
		if (!hrm) {
			re -= 64;
			hrm = lrm;
			lrm = 0;
		}
		/* adjust exponent */
		s = dclz(hrm) - 1;
		re -= (s-1);
		/* hrm of diff is 0 */
		if (s < 0)
			hrm = hrm >> (-s);
		else {
			hrm = (hrm << s) | (lrm >> (-s & 63));
			lrm = lrm << s;
			rm = hrm;
		}

		rm |= (lrm != 0);
	}
done:

	rm = (rm >> 7) | ((rm & 0x7f) != 0);

	return ieee754dp_format(rs, re, rm);
#else
	rm = hrm | (lrm != 0);

	/*
	 * Sticky shift down to normal rounding precision.
	 */
	if ((s64) rm < 0) {
		rm = (rm >> (64 - (DP_FBITS + 1 + 3))) |
		     ((rm << (DP_FBITS + 1 + 3)) != 0);
			re++;
	} else {
		rm = (rm >> (64 - (DP_FBITS + 1 + 3 + 1))) |
		     ((rm << (DP_FBITS + 1 + 3 + 1)) != 0);
	}
	assert(rm & (DP_HIDDEN_BIT << 3));

	/* And now the addition */
	assert(zm & DP_HIDDEN_BIT);

	/*
	 * Provide guard,round and stick bit space.
	 */
	zm <<= 3;

	if (ze > re) {
		/*
		 * Have to shift y fraction right to align.
		 */
		s = ze - re;
		rm = XDPSRS(rm, s);
		re += s;
	} else if (re > ze) {
		/*
		 * Have to shift x fraction right to align.
		 */
		s = re - ze;
		zm = XDPSRS(zm, s);
		ze += s;
	}
	assert(ze == re);
	assert(ze <= DP_EMAX);

	if (zs == rs) {
		/*
		 * Generate 28 bit result of adding two 27 bit numbers
		 * leaving result in xm, xs and xe.
		 */
		zm = zm + rm;

		if (zm >> (DP_FBITS + 1 + 3)) { /* carry out */
			zm = XDPSRS1(zm);
			ze++;
		}
	} else {
		if (zm >= rm) {
			zm = zm - rm;
		} else {
			zm = rm - zm;
			zs = rs;
		}
		if (zm == 0)
			return ieee754dp_zero(ieee754_csr.rm == FPU_CSR_RD);

		/*
		 * Normalize to rounding precision.
		 */
		while ((zm >> (DP_FBITS + 3)) == 0) {
			zm <<= 1;
			ze--;
		}
	}

	return ieee754dp_format(zs, ze, zm);
#endif
}


ieee754dp ieee754dp_maddf(ieee754dp z, ieee754dp x,
				ieee754dp y)
{
	return 	_ieee754dp_maddf(z, x, y, 0);
}

ieee754dp ieee754dp_nmaddf(ieee754dp z, ieee754dp x,
				ieee754dp y)
{
	return 	_ieee754dp_maddf(z, x, y, 1);
}
ieee754dp ieee754dp_msubf(ieee754dp z, ieee754dp x,
				ieee754dp y)
{
	return 	_ieee754dp_maddf(z, x, y, 2);
}

ieee754dp ieee754dp_nmsubf(ieee754dp z, ieee754dp x,
				ieee754dp y)
{
	return 	_ieee754dp_maddf(z, x, y, 3);
}
