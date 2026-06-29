import api from './client';
import type { AuthUser } from '../types';

// POST /auth/login — 이메일+비밀번호 검증, 성공 시 사용자 정보 반환
export async function login(email: string, password: string): Promise<AuthUser> {
  const res = await api.post<AuthUser>('/auth/login', { email, password });
  return res.data;
}
