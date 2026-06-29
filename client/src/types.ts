// API 응답 타입 정의 — server/schemas.py와 대응
// TypeScript가 API 응답 데이터의 속성을 타입 안전하게 접근할 수 있게 해줌

export type StorageType = 'chilled' | 'frozen';
export type OrderStatus = 'pending' | 'processing' | 'awaiting_pickup' | 'delivered' | 'cancelled';
export type CancelReason = 'user' | 'no_pickup' | 'misdelivery';
export type UserRole = 'customer' | 'admin';

export interface Product {
  name: string;
  storage_type: StorageType;
  stock: number;
}

export interface Order {
  id: number;
  user_id: number;
  product_name: string;
  status: OrderStatus;
  cancel_reason: CancelReason | null;
  created_at: string;
}

// 로그인/회원가입 후 앱 전역에서 사용하는 사용자 정보
export interface AuthUser {
  user_id: number;
  name: string;
  role: UserRole;
}
