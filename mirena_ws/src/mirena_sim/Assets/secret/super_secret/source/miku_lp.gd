extends Node3D

func _ready() -> void:
	$AudioStreamPlayer.stream = preload("res://Assets/secret/super_secret/quack.mp3")
	$AudioStreamPlayer.play()
	
	$GPUParticles3D.emitting = true
