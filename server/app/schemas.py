# Pydantic 스키마 — API 입출력 형식 정의
# 클라이언트가 보내는 요청 JSON과 서버가 반환하는 응답 JSON의 형태를 명시
# DB 테이블과 독립적 — 필요한 필드만 골라서 노출 가능
from datetime import datetime
from pydantic import BaseModel, ConfigDict
from app.models import OrderStatus, CancelReason, StorageType, UserRole


# ── 요청 스키마 (클라이언트 → 서버) ──────────────────────────────────────────

class LoginRequest(BaseModel):
    email: str
    password: str


class SignupRequest(BaseModel):
    email: str
    password: str
    name: str


class ProfileUpdate(BaseModel):
    # 이름 변경과 비밀번호 변경을 하나의 PATCH 엔드포인트로 처리
    # 전달된 필드만 업데이트, None인 필드는 무시
    name: str | None = None
    current_password: str | None = None
    new_password: str | None = None


class OrderCreate(BaseModel):
    user_id: int
    product_name: str


# ── 응답 스키마 (서버 → 클라이언트) ──────────────────────────────────────────

class LoginResponse(BaseModel):
    user_id: int
    name: str
    role: UserRole


# from_attributes=True: SQLAlchemy 객체를 Pydantic이 직접 읽을 수 있게 허용
# 없으면 return customer 같은 코드에서 에러 발생
class ProfileResponse(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    email: str
    name: str
    role: UserRole


class ProductResponse(BaseModel):
    name: str
    storage_type: StorageType
    stock: int  # DB 컬럼 아닌 집계값 (locations JOIN 결과)


class OrderResponse(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    user_id: int | None  # 회원탈퇴 후 NULL 가능
    product_name: str
    status: OrderStatus
    cancel_reason: CancelReason | None
    created_at: datetime
