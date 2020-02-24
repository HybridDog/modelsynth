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

local function get_label_key(data)
	return data[1] .. "," .. data[2] .. "," .. data[3]
	--~ return table.concat(data, ",")
end

local function vector_index(x, y, z, ystride, zstride)
	return z * zstride + y * ystride + x
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

	-- The 1x3x1 nodes sized labels in the areas
	local label_maps = {{}, {}, {}}

	-- Collect all labels and their positions
	for y_off = 0,2 do
		-- One label map for each offset in the y direction
		local label_map = label_maps[y_off+1]
		local y_start = pos1.y + y_off
		-- Go back two nodes in y direction so that the labels are searched
		-- only in the area between pos1 and pos2
		local y_end = pos2.y - 2
		local li = 0
		for z = pos1.z, pos2.z do
			for y = y_start, y_end, 3 do
				for x = pos1.x, pos2.x do
					-- The label consists of 1x3x1 nodes
					local vi_bottom = area:index(x, y, z)
					local vi_middle = vi_bottom + area.ystride
					local vi_top = vi_middle + area.ystride
					local label = {nodes[vi_bottom], nodes[vi_middle],
						nodes[vi_top]}
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
				end
			end
		end
	end

	-- Find neighbouring constraints
	local adjacencies = {x = {}, y = {}, z = {}}
	for y_off = 0,2 do
		local label_map = label_maps[y_off+1]
		local y_start = pos1.y + y_off
		local y_end = pos2.y - 2
		local label_ystride = pos2.x - pos1.x + 1
		-- The number of times the for loop along y was executed previously
		local label_zstride = math.floor((y_end - y_start) / 3) + 1
		-- One step along z means label_ystride along y
		label_zstride = label_zstride * label_ystride
		local li = 0
		for z = pos1.z, pos2.z-1 do
			for y = y_start, y_end-3, 3 do
				for x = pos1.x, pos2.x-1 do
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
				-- One x is skipped
				li = li + 1
			end
			-- One slice along x is skipped due to the the for loop end
			li = li + label_ystride
		end
	end

	-- Save node names so that the nodes in the labels are not ephemeral
	local id_to_nodename = {}
	for k = 0,num_labels-1 do
		local label = labels[k]
		for j = 1,3 do
			local nodeid = label[j]
			id_to_nodename[nodeid] = id_to_nodename[nodeid]
				or minetest.get_name_from_content_id(nodeid)
		end
	end

	-- Find the empty space and other special labels, this code is not yet
	-- flexible
	-- They are used for the intial model and boundary conditions
	local airid = minetest.get_content_id"air"
	local k = get_label_key{airid, airid, airid}
	local air_label_id = label_indices[k]
	if not air_label_id then
		minetest.log("warning", "No air empty space label found")
	end
	special_labels = {empty_space = air_label_id}

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
		self[ci * self.num_labels + label] = 0
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
				if next_stride ~= stride then
					arcs_queue:add{ci, next_stride}
				end
			end
		end
	end
end

local function get_index_function(ystride, zstride)
	return function(x, y, z)
		return z * zstride + y * ystride + x
	end
end

-- Returns false if it fails
local function generate_chunk(modelinfo, index_model, region, model)
	local num_labels = modelinfo.num_labels
	-- Start and end positions in the region
	local p1 = region[1]
	local p2 = region[2]
	-- Save the current model in the region for the failure case
	local model_backup = {}
	for z = p1[3], p2[3] do
		for y = p1[2], p2[2] do
			local mi = index_model(p1[1], y, z)
			for x = p1[1], p2[1] do
				model_backup[#model_backup+1] = model[mi]
				mi = mi+1
			end
		end
	end
	-- The catalog needs to be 1 bigger than the region in all directions
	local wc = p2[1] - p1[1] + 1 + 2
	local hc = p2[2] - p1[2] + 1 + 2
	local lc = p2[3] - p1[3] + 1 + 2
	local function index_model_off(cx, cy, cz)
		return index_model(p1[1]-1 + cx, p1[2]-1 + cy, p1[3]-1 + cz)
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
				-- Add arcs from all possible neighbours
				-- FIXME: actually only arcs from the border to the inside are
				-- needed
				local ci = vector_index(cx, cy, cz, wc, hc * wc)
				for i = 1,#catalog_strides do
					local stride = catalog_strides[i]
					arcs[num_arcs+1] = {ci - stride, stride}
					num_arcs = num_arcs + 1
				end
			end
		end
	end
	local arcs_queue = datastructures.create_queue{input = arcs}
	-- Here the catalog should have at least one label for each position
	apply_neighbour_constraints(catalog, arcs_queue)

	-- Do the Discrete Model Synthesis in the region
	for z = 1, lc-2 do
		for y = 1, hc-2 do
			-- Index for the model
			local mi = index_model_off(1, y, z)
			-- Index for the catalog
			local ci = vector_index(1, y, z, wc, hc * wc)
			for x = 1, wc-2 do
				local chosen_label = catalog:choose_random_label(ci)
				if not chosen_label then
					-- It failed, so reset the model to its backup
					local backup_i = 1
					for z = p1[3], p2[3] do
						for y = p1[2], p2[2] do
							local mi = index_model(p1[1], y, z)
							for x = p1[1], p2[1] do
								model[mi] = model_backup[backup_i]
								backup_i = backup_i+1
								mi = mi+1
							end
						end
					end
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
	local w = pos2.x - pos1.x
	local h = pos2.y - pos1.y
	if h % 3 > 0 then
		minetest.log("warning", "Height is not a multiple of 3")
		h = h - h % 3
	end
	local l = pos2.z - pos1.z

	-- Generate the model
	-- Use bigger width and height for the boundary conditions
	local wo = w + 2
	local ho = h + 2
	local lo = l + 2
	local model = {}
	-- Fill everything with empty space
	local empty_space = modelinfo.special_labels.empty_space
	for i = 0, wo * ho * lo - 1 do
		model[i] = empty_space
	end
	-- Subdivide the to-be-generated model into overlapping smaller regions
	local regions = datastructures.create_queue()
	local region_size_max = 2 * 16
	for z = 1, lo-2, region_size_max / 2 do
		for y = 1, ho-2, region_size_max / 2 do
			for x = 1, wo-2, region_size_max / 2 do
				local p1 = {x, y, z}
				local p2 = {
					math.max(x + region_size_max-1, wo-2),
					math.max(y + region_size_max-1, ho-2),
					math.max(z + region_size_max-1, lo-2),
				}
				regions:add{p1, p2}
			end
		end
	end
	assert(not regions:is_empty())
	-- Function which returns an index in the model array
	local index_model = get_index_function(wo, ho * wo)
	-- Generate in each region, add more regions if it failed
	repeat
		local region = regions:take()
		log("Trying to generate region " .. dump_region(region))
		if not generate_chunk(modelinfo, index_model, region, model) then
			log("Failed, subdividing…")
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
				log("New regions:")
				for i = 1,#new_regions do
					regions:add(new_regions[i])
					log(dump_region(new_regions[i]))
				end
			end
		end
	until regions:is_empty()

	-- Put the nodes
	local vm = minetest.get_voxel_manip()
	local e1, e2 = vm:read_from_map(pos1, pos2)
	local area = VoxelArea:new{MinEdge=e1, MaxEdge=e2}
	local nodes = vm:get_data()

	-- Prepare model pieces (labels)
	local old_to_new_contentid = {}
	for i, nodename in pairs(modelinfo.nodeids) do
		old_to_new_contentid[i] = minetest.get_content_id(nodename)
	end
	local labels_nodes = {}
	for i = 0, num_labels do
		local label = modelinfo.labels[i]
		labels_nodes[i] = {old_to_new_contentid[label[1]],
			old_to_new_contentid[label[2]], old_to_new_contentid[label[3]]}
	end

	for z = 1, lo-2 do
		for y = 1, ho-2 do
			local mi = index_model(1, y, z)
			local vi = area:index(pos1.x, pos1.y + (y - 1) * 3, pos1.z + z - 1)
			for x = 1, wo-2 do
				local label_nodes = labels_nodes[model[mi]]
				nodes[vi] = label_nodes[1]
				nodes[vi + area.ystride] = label_nodes[2]
				nodes[vi + 2 * area.ystride] = label_nodes[3]
				old_to_new_contentid[i]
				mi = mi+1
				vi = vi+1
			end
		end
	end

	vm:set_data(nodes)
	vm:write_to_map()
end

local version_head = "modelsynth_v0\n"

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

		generate_model(pos1, pos2, modelinfo)

		return true
	end,
})

