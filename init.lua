local path = minetest.get_modpath"modelsynth"
local datastructures = dofile(path .. "/datastructures.lua")

local function dump_region(r)
	return ("Region (%d - %d, %d - %d, %d - %d)"):format(r[1][1], r[2][1],
		r[1][2], r[2][2], r[1][3], r[2][3])
end
local function log(msg)
	print(msg)
	minetest.chat_send_all(msg)
end
local function log_debug(msg)
	log(msg)
end
-- I've commented log_verbose calls to avoid redundant string generation;
-- unfortunately there's no #ifdef in lua.
local function log_verbose(msg)
	log_debug(msg)
end

local function get_label_key(data)
	return table.concat(data, ",")
end

-- Designed for 2x3x2 sized model pieces; the entire model must fit into the
-- area defined by pos1 and pos2
local function get_model_size(pos1, pos2)
	local s = vector.add(vector.subtract(pos2, pos1), 1)
	return math.floor(s.x / 2), math.floor(s.y / 3), math.floor(s.z / 2)
end

-- Returns VoxelArea offsets for index offsets within a label and offsets to
-- move to the start of the next label
local function get_label_strides(area)
	local offsets = {}
	-- Order by y first
	for y = 0, 2 do
		for z = 0, 1 do
			for x = 0,1 do
				offsets[#offsets+1] = z * area.zstride + y * area.ystride + x
			end
		end
	end
	local label_skip_offsets = {x = 2, y = 3 * area.ystride,
		z = 2 * area.zstride}
	return offsets, label_skip_offsets
end

local function get_empty_label(label_indices)
	-- This code is not flexible, but air almost always represents empty space.
	local airid = minetest.get_content_id"air"
	local label_air = {}
	for i = 1, 2*3*2 do
		label_air[i] = airid
	end
	local k = get_label_key(label_air)
	local air_label_id = label_indices[k]
	if not air_label_id then
		minetest.log("warning", "No air empty space label found")
	end
	return air_label_id
end

-- Calculates placeable model pieces from the labels
local function prepare_model_pieces(modelinfo)
	local old_to_new_contentid = {}
	for i, nodename in pairs(modelinfo.nodeids) do
		old_to_new_contentid[i] = minetest.get_content_id(nodename)
	end
	local labels_nodes = {}
	for i = 0, modelinfo.num_labels-1 do
		local label = modelinfo.labels[i]
		local newlabel = {}
		for k = 1, 2*3*2 do
			newlabel[k] = old_to_new_contentid[label[k]]
		end
		labels_nodes[i] = newlabel
	end
	return labels_nodes
end

local function generate_modelinfo(pos1, pos2)
	-- Labels are model pieces, indices start from zero
	local labels = {}
	local label_indices = {}
	local num_labels = 0

	local vm = minetest.get_voxel_manip()
	local e1, e2 = vm:read_from_map(pos1, pos2)
	local area = VoxelArea:new{MinEdge=e1, MaxEdge=e2}
	local nodes = vm:get_data()

	-- Firstly, collect all labels and their positions, i.e. the input model

	-- The 2x3x2 nodes sized labels in the area
	local label_map = {}

	local wm, hm, lm = get_model_size(pos1, pos2)
	local label_strides, label_skips = get_label_strides(area)
	local li = 0
	local vi_start = area:indexp(pos1)
	for z = 0, lm-1 do
		for y = 0, hm-1 do
			local vi = vi_start + z * label_skips.z + y * label_skips.y
			for x = 0, wm-1 do
				local label = {}
				for i = 1,#label_strides do
					label[#label+1] = nodes[vi + label_strides[i]]
				end
				-- Add the label if it isn't known yet
				local k = get_label_key(label)
				local i = label_indices[k]
				if not i then
					-- Add this newly found label
					i = num_labels
					label_indices[k] = i
					labels[i] = label
					num_labels = num_labels+1
				end
				label_map[li] = i
				li = li+1
				vi = vi + label_skips.x
			end
		end
	end

	-- Find neighbouring constraints
	local adjacencies = {x = {}, y = {}, z = {}}
	local label_ystride = wm
	local label_zstride = hm * wm
	local li = 0
	for z = 0, lm-2 do
		for y = 0, hm-2 do
			for x = 0, wm-2 do
				-- Only +x, +y, +z transitions are saved because of symmetry
				local label_current = label_map[li]
				local label_z = label_map[li + label_zstride]
				local label_y = label_map[li + label_ystride]
				local label_x = label_map[li + 1]
				adjacency_i = label_current * num_labels
				adjacencies.z[adjacency_i + label_z] = true
				adjacencies.y[adjacency_i + label_y] = true
				adjacencies.x[adjacency_i + label_x] = true
				li = li+1
			end
			-- Skip one x because of the x for loop end
			li = li + 1
		end
		-- Skip a slice along x because of the y for loop end
		li = li + label_ystride
	end

	-- Save node names so that the nodes in the labels are not ephemeral
	local id_to_nodename = {}
	for k = 0, num_labels-1 do
		local label = labels[k]
		for j = 1, #label do
			local nodeid = label[j]
			id_to_nodename[nodeid] = id_to_nodename[nodeid]
				or minetest.get_name_from_content_id(nodeid)
		end
	end

	-- Find the empty space and other special labels.
	-- I use special labels for the intial model and boundary conditions.
	special_labels = {empty_space = get_empty_label(label_indices)}

	return {
		num_labels = num_labels,
		labels = labels,
		nodeids = id_to_nodename,
		adjacencies = adjacencies,
		special_labels = special_labels,
	}
end

local Catalog = {}
setmetatable(Catalog, {__call = function(_, width, height, length, modelinfo)
	local num_labels = modelinfo.num_labels
	local obj = {
		w = width,
		h = height,
		l = length,
		wh = width * height,
		num_labels = num_labels,
	}
	local adjacencies = modelinfo.adjacencies
	local strides = {x = 1, y = obj.w, z = obj.wh}
	obj.all_strides = {strides.x, -strides.x, strides.y, -strides.y, strides.z,
		-strides.z}

	local adjacency_testers = {}
	-- The adjacency matrices for the various strides
	local stride_to_adjacencies = {
		[strides.x] = adjacencies.x,
		[-strides.x] = adjacencies.x,
		[strides.y] = adjacencies.y,
		[-strides.y] = adjacencies.y,
		[strides.z] = adjacencies.z,
		[-strides.z] = adjacencies.z
	}
	for stride, adjacencies in pairs(stride_to_adjacencies) do
		if stride < 0 then
			-- Negative direction
			adjacency_testers[stride] = function(label2, label1)
				return adjacencies[num_labels * label1 + label2]
			end
		else
			adjacency_testers[stride] = function(label1, label2)
				return adjacencies[num_labels * label1 + label2]
			end
		end
	end
	obj.adjacency_testers = adjacency_testers

	setmetatable(obj, Catalog)
	return obj
end})
Catalog.__index = {
	pos_index = function(self, x, y, z)
		return z * self.wh + y * self.w + x
	end,

	label_allowed = function(self, ci, label)
		return self[ci * self.num_labels + label]
	end,

	-- Returns true iff at least one label is allowed at ci
	some_label_allowed = function(self, ci)
		for l = 0, self.num_labels-1 do
			if self:label_allowed(ci, l) then
				return true
			end
		end
		return false
	end,

	-- Returns an iterator over the current domain of the variable at ci
	iter_label_alloweds = function(self, ci)
		local l = -1
		local l_max = self.num_labels-1
		return function()
			for newl = l+1, l_max do
				if self:label_allowed(ci, newl) then
					l = newl
					return l
				end
			end
			-- return nil
		end
	end,

	get_strides = function(self)
		-- The strides represent the direction, e.g. +X, -X
		return self.all_strides
	end,

	-- Returns a function which tests if two labels can be adjacent
	-- in the direction of a specified stride
	get_adjacency_tester = function(self, stride)
		return self.adjacency_testers[stride]
	end,

	-- Returns a random label from all remaining possible labels at ci
	-- and removes all other labels at ci
	choose_random_label = function(self, ci)
		local possible_labels = {}
		for label in self:iter_label_alloweds(ci) do
			possible_labels[#possible_labels+1] = label
		end
		if #possible_labels == 0 then
			return
		end
		local chosen_label = possible_labels[math.random(#possible_labels)]
		for i = 1, #possible_labels do
			local label = possible_labels[i]
			if label ~= chosen_label then
				self:remove_label(ci, label)
			end
		end
		return chosen_label
	end,

	remove_label = function(self, ci, label)
		self[ci * self.num_labels + label] = nil
	end,

	remove_label_at = function(self, x, y, z, label)
		self:remove_label(self:pos_index(x,y,z), label)
	end,

	-- Add all labels to all positions in the catalog
	init_full = function(self)
		for i = 0, self.w * self.h * self.l * self.num_labels - 1 do
			self[i] = true
		end
	end,
}

local Model = {}
setmetatable(Model, {__call = function(_, width, height, length, modelinfo)
	local obj = {
		w = width,
		h = height,
		l = length,
		wh = width * height,
		empty_space = modelinfo.special_labels.empty_space,
		--~ modelinfo = modelinfo,
	}

	setmetatable(obj, Model)
	return obj

end})
Model.__index = {
	index = function(self, x, y, z)
		return z * self.wh + y * self.w + x
	end,

	get_backup = function(self, p1, p2)
		-- Save the current model in the region for the failure case
		-- Only the area defined by p1 and p2 is relevant
		local model_backup = {true}
		local bi = 0
		for z = p1[3], p2[3] do
			for y = p1[2], p2[2] do
				local mi = self:index(p1[1], y, z)
				for x = p1[1], p2[1] do
					model_backup[bi] = self[mi]
					mi = mi+1
					bi = bi+1
				end
			end
		end
		model_backup.p1 = p1
		model_backup.p2 = p2
		return model_backup
	end,

	-- Fills the whole model with empty space, which makes it fulfill the
	-- adjacency constraints trivially
	fill_empty = function(self)
		for i = 0, self.wh * self.l - 1 do
			self[i] = self.empty_space
		end
	end,

	apply_backup = function(self, model_backup)
		local p1 = model_backup.p1
		local p2 = model_backup.p2
		local bi = 0
		for z = p1[3], p2[3] do
			for y = p1[2], p2[2] do
				local mi = self:index(p1[1], y, z)
				for x = p1[1], p2[1] do
					self[mi] = model_backup[bi]
					bi = bi+1
					mi = mi+1
				end
			end
		end
	end,
}

-- Removes invalid labels from a single catalog entry,
-- returns false if nothing has changed
local function remove_invalid_labels(ci_prev, ci, test_adjacency,
		catalog)
	local have_changes = false
	for label in catalog:iter_label_alloweds(ci) do
		-- Test if there is a label at ci_prev which allows `label` to be
		-- at ci
		local label_valid = false
		for label_prev in catalog:iter_label_alloweds(ci_prev) do
			if test_adjacency(label_prev, label) then
				label_valid = true
				break
			end
		end
		-- Remove the label from the catalog if it is not allowed
		if not label_valid then
			catalog:remove_label(ci, label)
			have_changes = true
		end
	end
	return have_changes
end

-- An implementation of Arc Consistency
local function apply_neighbour_constraints(catalog, arcs_queue)
	local strides = catalog:get_strides()
	while not arcs_queue:is_empty() do
		local arc = arcs_queue:take()
		local ci_prev = arc[1]
		local stride = arc[2]
		local ci = ci_prev + stride
		local test_adjacency = catalog:get_adjacency_tester(stride)
		if remove_invalid_labels(ci_prev, ci, test_adjacency,
				catalog) then
			-- FIXME: this resembles Flood Fill, there may be a faster algorithm
			-- Labels were removed, so continue with neighbours of ci
			for i = 1,#strides do
				local next_stride = strides[i]
				-- Do not go back
				if next_stride ~= -stride then
					arcs_queue:add{ci, next_stride}
				end
			end
		end
	end
end

-- Returns false if it fails
local function generate_chunk(modelinfo, region, model)
	local num_labels = modelinfo.num_labels
	-- Start and end positions in the region
	local p1 = region[1]
	local p2 = region[2]
	-- Save the current model in the region for the failure case
	local model_backup = model:get_backup(p1, p2)
	-- The catalog needs to be 1 bigger than the region in all directions
	local wc = p2[1] - p1[1] + 1 + 2
	local hc = p2[2] - p1[2] + 1 + 2
	local lc = p2[3] - p1[3] + 1 + 2
	-- This function indices the model with a position in the catalog
	-- (it needs to be translated)
	local function index_model_off(cx, cy, cz)
		return model:index(p1[1]-1 + cx, p1[2]-1 + cy, p1[3]-1 + cz)
	end
	-- Initialize catalog
	local catalog = Catalog(wc, hc, lc, modelinfo)
	catalog:init_full()
	-- Remove labels from the border, where the model is fixed
	-- FIXME: this can be done more efficiently
	for cz = 0, lc-1 do
		for cy = 0, hc-1 do
			for cx = 0, wc-1 do
				if (cz == 0 or cz == lc-1) or (cy == 0 or cy == hc-1)
						or (cx == 0 or cx == wc-1) then
					local fixed_label = model[index_model_off(cx, cy, cz)]
					for label = 0, num_labels-1 do
						if label ~= fixed_label then
							catalog:remove_label_at(cx, cy, cz, label)
						end
					end
				end
			end
		end
	end
	-- The preprocessing step: remove invalid labels from the catalog
	local catalog_strides = catalog:get_strides()
	local arcs = {}
	local num_arcs = 0
	-- Arcs to the border of the catalog don't need to be added
	for cz = 1, lc - 2 do
		for cy = 1, hc - 2 do
			for cx = 1, wc - 2 do
				-- Add arcs only from the border to the inside
				if cz == 1 or cy == 1 or cx == 1 then
					local ci = catalog:pos_index(cx, cy, cz)
					for i = 1,#catalog_strides do
						local stride = catalog_strides[i]
						if stride > 0 then
							arcs[num_arcs+1] = {ci - stride, stride}
							num_arcs = num_arcs + 1
						end
					end
				elseif cz == lc - 2 or cy == hc - 2 or cx == wc - 2 then
					local ci = catalog:pos_index(cx, cy, cz)
					for i = 1,#catalog_strides do
						local stride = catalog_strides[i]
						if stride < 0 then
							arcs[num_arcs+1] = {ci - stride, stride}
							num_arcs = num_arcs + 1
						end
					end
				end
			end
		end
	end
	local arcs_queue = datastructures.create_queue{input = arcs}
	apply_neighbour_constraints(catalog, arcs_queue)
	-- Here the catalog should have at least one label for each position
	if not catalog:some_label_allowed(catalog:pos_index(1, 1, 1)) then
		log("Error: No label allowed")
		return false
	end

	-- Do the Discrete Model Synthesis in the region
	for z = 1, lc-2 do
		for y = 1, hc-2 do
			-- Index for the model
			local mi = index_model_off(1, y, z)
			-- Index for the catalog
			local ci = catalog:pos_index(1, y, z)
			for x = 1, wc-2 do
				local chosen_label = catalog:choose_random_label(ci)
				if not chosen_label then
					-- It failed, so reset the model to its backup
					model:apply_backup(model_backup)
					return false
				end

				model[mi] = chosen_label

				-- Remove labels which became invalid because a label has been
				-- chosen for position ci
				local strides = catalog:get_strides()
				for i = 1,#strides do
					arcs_queue:add{ci, strides[i]}
				end
				apply_neighbour_constraints(catalog, arcs_queue)

				mi = mi+1
				ci = ci+1
			end
		end
	end

	return true
end

local function copy_region(region)
	return {
		{region[1][1], region[1][2], region[1][3]},
		{region[2][1], region[2][2], region[2][3]},
	}
end

local function generate_model(pos1, pos2, modelinfo)
	assert(type(modelinfo) == "table", "invalid modelinfo argument")
	-- Generate the model
	local wm, hm, lm = get_model_size(pos1, pos2)
	-- Use bigger width and height for the boundary conditions
	wm = wm + 2
	hm = hm + 2
	lm = lm + 2
	local model = Model(wm, hm, lm, modelinfo)
	log("Model size: " .. wm .. "x" .. hm .. "x" .. lm)
	-- Fill everything with empty space
	model:fill_empty()
	-- Subdivide the to-be-generated model into overlapping smaller regions
	local regions = datastructures.create_queue()
	local region_size_max = 2 * 16
	for z = 1, lm-2, region_size_max / 2 do
		for y = 1, hm-2, region_size_max / 2 do
			for x = 1, wm-2, region_size_max / 2 do
				local p1 = {x, y, z}
				local p2 = {
					math.min(x + region_size_max-1, wm-2),
					math.min(y + region_size_max-1, hm-2),
					math.min(z + region_size_max-1, lm-2),
				}
				regions:add{p1, p2}
			end
		end
	end
	if regions:is_empty() then
		log("Error: no regions")
	end
	-- Generate in each region, add more regions if it failed
	repeat
		local region = regions:take()
		--~ log_verbose("Trying to generate region " .. dump_region(region))
		if not generate_chunk(modelinfo, region, model) then
			--~ log_verbose("Failed to generate " .. dump_region(region) ..
				--~ ", subdividing…")
			-- It failed, so subdivide the region
			local p1 = region[1]
			local p2 = region[2]
			-- Split into 3*3*3 = 27 new overlapping regions
			local new_regions = {region}
			for splitdir = 1,3 do
				local len = p2[splitdir] - p1[splitdir] + 1
				if len >= 4 then
					local newlen = math.ceil(len * 0.5)
					local stride_mid = math.floor(newlen * 0.5)
					local starts = {p1[splitdir], p1[splitdir] + stride_mid,
						p1[splitdir] + newlen}
					local ends = {starts[1] + newlen-1, starts[2] + newlen-1,
						p2[splitdir]}
					local previous_split = new_regions
					new_regions = {}
					for i = 1,#previous_split do
						local region = previous_split[i]
						for k = 1,3 do
							region = copy_region(region)
							region[1][splitdir] = starts[k]
							region[2][splitdir] = ends[k]
							new_regions[#new_regions+1] = region
						end
					end
				end
			end
			if #new_regions > 1 then
				--~ log_verbose("New regions:")
				for i = 1,#new_regions do
					regions:add(new_regions[i])
					--~ log_verbose(dump_region(new_regions[i]))
				end
			end
		end
	until regions:is_empty()

	log_debug("Putting the nodes…")
	local vm = minetest.get_voxel_manip()
	local e1, e2 = vm:read_from_map(pos1, pos2)
	local area = VoxelArea:new{MinEdge=e1, MaxEdge=e2}
	local nodes = vm:get_data()

	local labels_nodes = prepare_model_pieces(modelinfo)
	local label_strides, label_skips = get_label_strides(area)

	local vi_start = area:indexp(pos1)
	for z = 1, lm-2 do
		for y = 1, hm-2 do
			local mi = model:index(1, y, z)
			local vi = vi_start + (z - 1) * label_skips.z
				+ (y - 1) * label_skips.y
			for x = 1, wm-2 do
				-- Place the content of the label
				local label_nodes = labels_nodes[model[mi]]
				for k = 1, #label_strides do
					nodes[vi + label_strides[k]] = label_nodes[k]
				end
				mi = mi+1
				vi = vi + label_skips.x
			end
		end
	end

	vm:set_data(nodes)
	vm:write_to_map()

	log("Done.")
end

local version_head = "modelsynth_v1\n"

-- Serialize the table to a string to save it on disk
local function encode_modelinfo(modelinfo)
	-- Shallow copy
	local output = {}
	for k,v in pairs(modelinfo) do
		output[k] = v
	end

	-- Convert adjacencies to a list for more compact serialization
	output.adjacencies = nil
	local adj_lists = {}
	for dir, adjacencies in pairs(modelinfo.adjacencies) do
		local list = {}
		for label in pairs(adjacencies) do
			list[#list+1] = label
		end
		adj_lists[dir] = list
	end
	output.adjacencies_lists = adj_lists

	return version_head ..
		minetest.serialize(output)
end

-- Used when loading a modelinfo from disk
local function decode_modelinfo(data)
	if data:sub(1, #version_head) ~= version_head then
		return nil, "invalid version"
	end
	data = minetest.deserialize(data:sub(#version_head+1))

	-- Shallow copy
	local modelinfo = {}
	for k,v in pairs(data) do
		modelinfo[k] = v
	end

	modelinfo.adjacencies_lists = nil
	local adjacencies = {}
	for dir, list in pairs(data.adjacencies_lists) do
		local adjs = {}
		for i = 1,#list do
			adjs[list[i]] = true
		end
		adjacencies[dir] = adjs
	end
	modelinfo.adjacencies = adjacencies

	return modelinfo
end

worldedit.register_command("gen_mi", {
	params = "filename",
	description = "[modelsynth] Generate the model information and save " ..
		"it to <filename>",
	privs = {worldedit=true},
	require_pos = 2,
	parse = function(param)
		return true, param:trim()
	end,
	func = function(playername, filename)
		local pos1, pos2 = worldedit.sort_pos(worldedit.pos1[playername],
			worldedit.pos2[playername])
		local modelinfo = generate_modelinfo(pos1, pos2)
		local output_data = encode_modelinfo(modelinfo)

		local path = minetest.get_worldpath() .. "/modelsynth"
		-- Create directory if needed
		minetest.mkdir(path)

		local filename = path .. "/" .. filename .. ".dat"
		local file, err = io.open(filename, "wb")
		if not file then
			minetest.chat_send_player(playername, "Cannot save to \"" ..
				filename .. "\": " .. err)
			return false
		end
		file:write(output_data)
		file:flush()
		file:close()

		minetest.chat_send_player(playername, "Saved modelinfo to \"" ..
			filename .. "\"")
		return true
	end,
})

local function print_modelinfo(modelinfo)
	print(dump(modelinfo))
	print("Labels:")
	local ids = modelinfo.nodeids
	for i = 0, modelinfo.num_labels-1 do
		local l = modelinfo.labels[i]
		local s = i .. ": "
		for k = 1,#l do
			s = s .. ids[l[k]] .. ", "
		end
		print(s)
	end
	print("Adjacencies:")
	for dir, t in pairs(modelinfo.adjacencies) do
		print("Direction " .. dir .. ":")
		for k in pairs(t) do
			local label1 = math.floor(k / modelinfo.num_labels)
			local label2 = k % modelinfo.num_labels
			print(("%d - %d"):format(label1, label2))
		end
	end
end

worldedit.register_command("synth", {
	params = "filename",
	description = "[modelsynth] Synthesize a model from the model " ..
		"information in <filename>",
	privs = {worldedit=true},
	require_pos = 2,
	parse = function(param)
		return true, param:trim()
	end,
	func = function(playername, filename)
		local pos1, pos2 = worldedit.sort_pos(worldedit.pos1[playername],
			worldedit.pos2[playername])

		local path = minetest.get_worldpath() .. "/modelsynth"
		local filename = path .. "/" .. filename .. ".dat"
		local file, err = io.open(filename, "rb")
		if not file then
			minetest.chat_send_player(playername, "Cannot load from \"" ..
				filename .. "\": " .. err)
			return false
		end
		local data = file:read"*a"
		file:close()

		local modelinfo, errormsg = decode_modelinfo(data)
		if not modelinfo then
			minetest.chat_send_player(playername, "Cannot load modelinfo: " ..
				errormsg)
		end
		print_modelinfo(modelinfo)

		generate_model(pos1, pos2, modelinfo)

		return true
	end,
})

-- TODO: maybe use AC2001?,
-- it always generates air now…
