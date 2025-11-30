package com.example.simulator.math

interface Path {
    fun isFollowable(): Boolean
    fun length(): Double
}