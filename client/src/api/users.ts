import api from './client';
import type { AuthUser } from '../types';

export interface Profile {
  id: number;
  email: string;
  name: string;
  role: AuthUser['role'];
}

// POST /auth/signup — 회원가입 후 자동 로그인 (AuthUser 반환)
export async function signup(email: string, password: string, name: string): Promise<AuthUser> {
  const res = await api.post<AuthUser>('/auth/signup', { email, password, name });
  return res.data;
}

// GET /users/:id — 프로필 조회
export async function fetchProfile(userId: number): Promise<Profile> {
  const res = await api.get<Profile>(`/users/${userId}`);
  return res.data;
}

// PATCH /users/:id — 이름 변경 또는 비밀번호 변경 (전달된 필드만 업데이트)
export async function updateProfile(
  userId: number,
  data: { name?: string; current_password?: string; new_password?: string },
): Promise<Profile> {
  const res = await api.patch<Profile>(`/users/${userId}`, data);
  return res.data;
}

// DELETE /users/:id — 회원탈퇴 (진행 중 주문 있으면 서버에서 409 반환)
export async function deleteAccount(userId: number): Promise<void> {
  await api.delete(`/users/${userId}`);
}
